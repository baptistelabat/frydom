// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#include "FrHydroMesh.h"

#include "frydom/core/body/FrBody.h"

namespace frydom {

    FrHydroMesh::FrHydroMesh(FrOffshoreSystem *system, std::string meshfile, std::shared_ptr<FrBody> body,
                             bool WNL_or_NL)
                             : m_system(system), m_meshfilename(meshfile), m_body(body), m_WNL_or_NL(WNL_or_NL) {
        // m_clipper
        m_clipper = std::make_unique<mesh::MeshClipper>();

        m_meshOffset = FrFrame();

    }

    void FrHydroMesh::Initialize() {

        // This function initializes the hydrostatic force object.

        // Loading the input mesh file.
        m_initMesh = mesh::FrMesh(m_meshfilename);
        m_clippedMesh = mesh::FrMesh();

        // Tidal height.
        double TidalHeight = m_system->GetEnvironment()->GetOcean()->GetFreeSurface()->GetTidal()->GetHeight(NWU);

        // Clipping surface.
        if(m_WNL_or_NL) { // Incident wave field.

            // Incident free surface.
            FrFreeSurface *FreeSurface = m_system->GetEnvironment()->GetOcean()->GetFreeSurface();

            // Setting the free surface.
            m_clipper->SetWaveClippingSurface(TidalHeight, FreeSurface);
        }
        else{ // Plane.

            // Setting the free surface.
            m_clipper->SetPlaneClippingSurface(TidalHeight);
        }

        // Body.
        m_clipper->SetBody(m_body.get());

//        // Position and orientation of the mesh frame compared to the body frame.
//        m_clipper->SetMeshOffsetRotation(m_MeshOffset, m_Rotation);

        // Initialization of the parent class.
        FrPrePhysicsItem::Initialize();
        m_chronoPhysicsItem->SetupInitial();

    }

    void FrHydroMesh::Compute(double time) {

        m_clippedMesh.clear();
        m_clippedMesh = m_initMesh;

        // Adjust the position of the clipped mesh according to the position of the body
        UpdateMeshPositionInWorld();

        // This function computes the nonlinear hydrostatic loads.

        m_clippedMesh = m_clipper->Apply(m_clippedMesh);
    }

    void FrHydroMesh::UpdateMeshPositionInWorld() {

        // This function transports the mesh from the mesh frame to the body frame, then applies the rotation of mesh in the world frame.

        // Iterating on vertices to get their place wrt to plane.
        VertexHandle vh;
        Position NodeInBody, NodeInWorld;
        Position BodyPos = m_body->GetPosition(NWU);

        // Loop over the vertices.
        for (mesh::FrMesh::VertexIter vh_iter = m_clippedMesh.vertices_begin();
             vh_iter != m_clippedMesh.vertices_end(); ++vh_iter) {

            vh = *vh_iter;

            // From the mesh frame to the body frame.
            m_clippedMesh.point(vh) = GetNodePositionInBody(m_clippedMesh.point(vh));

            NodeInBody[0] = m_clippedMesh.point(vh)[0];
            NodeInBody[1] = m_clippedMesh.point(vh)[1];
            NodeInBody[2] = m_clippedMesh.point(vh)[2];

            // Rotation from the body frame to the world frame (just the rotation and the vertical translation, not the horizontal translation of the mesh at the good position in the world mesh).
            // The horizontal translation is not done to avoid numerical errors.
            NodeInWorld = m_body->ProjectVectorInWorld<Position>(NodeInBody, NWU);

            // Vertical translation.
            NodeInWorld[2] = NodeInWorld[2] + BodyPos[2]; // x.

            m_clippedMesh.point(vh)[0] = NodeInWorld[0];
            m_clippedMesh.point(vh)[1] = NodeInWorld[1];
            m_clippedMesh.point(vh)[2] = NodeInWorld[2];

        }

    }

    VectorT<double, 3> FrHydroMesh::GetNodePositionInBody(VectorT<double, 3> point) const {

        // From the mesh frame to the body frame: OmP = ObOm + bRm*OmP.
        mathutils::Vector3d<double> NodeInMeshFrameVect;
        NodeInMeshFrameVect[0] = point[0];
        NodeInMeshFrameVect[1] = point[1];
        NodeInMeshFrameVect[2] = point[2];

        mathutils::Vector3d<double> TmpVect;
        TmpVect = m_meshOffset.GetRotation().GetRotationMatrix()*NodeInMeshFrameVect;

        Position NodeInBodyFrame;
        NodeInBodyFrame[0] = m_meshOffset.GetPosition(NWU)[0] + TmpVect[0];
        NodeInBodyFrame[1] = m_meshOffset.GetPosition(NWU)[1] + TmpVect[1];
        NodeInBodyFrame[2] = m_meshOffset.GetPosition(NWU)[2] + TmpVect[2];

        // Position -> point.
        mesh::FrMesh::Point Pout;
        Pout[0] = NodeInBodyFrame[0];
        Pout[1] = NodeInBodyFrame[1];
        Pout[2] = NodeInBodyFrame[2];

        return Pout;

    }

    Position FrHydroMesh::GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc){

        // This function returns the center of buoyancy of the clipped mesh in the world frame.

        Position CoBInWorld, CoBPos;
        VectorT<double, 3> CoB = m_clippedMesh.GetCOG(); // Center of gravity of the immersed part (clipped mesh).
        CoBPos[0] = CoB[0];
        CoBPos[1] = CoB[1];
        CoBPos[2] = CoB[2];

        // The translation of the body was not done for avoiding numerical errors.
        CoBInWorld = m_body->GetPosition(NWU) + CoBPos;

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(CoBInWorld);

        return CoBInWorld;
    }

    void FrHydroMesh::InitializeLog() {

    }

    void FrHydroMesh::SetMeshOffset(FrFrame meshOffset) {
        m_meshOffset = meshOffset;
    }

    mesh::FrMesh FrHydroMesh::GetClippedMesh() {
        return m_clippedMesh;
    }

    mesh::FrMesh FrHydroMesh::GetInitialMesh() {
        return m_initMesh;
    }

    std::shared_ptr<FrHydroMesh> make_hydro_mesh_nonlinear(std::shared_ptr<FrBody> body, std::string meshfile){

        // This function creates a hydrodynamic mesh for using in the computation of the nonlinear hydrostatic and/or Froude-Krylov loads.

        auto HydroMesh = std::make_shared<FrHydroMesh>(body->GetSystem(),meshfile,body,true);

        body->GetSystem()->Add(HydroMesh);

        return HydroMesh;
    }

    std::shared_ptr<FrHydroMesh> make_hydro_mesh_weakly_nonlinear(std::shared_ptr<FrBody> body, std::string meshfile){

        // This function creates a hydrodynamic mesh for using in the computation of the weakly nonlinear hydrostatic and/or Froude-Krylov loads.

        auto HydroMesh = std::make_shared<FrHydroMesh>(body->GetSystem(),meshfile,body,false);

        body->GetSystem()->Add(HydroMesh);

        return HydroMesh;
    }

}  // end namespace frydom