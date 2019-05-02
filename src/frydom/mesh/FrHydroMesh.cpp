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

        // Set the body position for horizontal correction in the clipping surface
        m_clipper->GetClippingSurface()->SetBodyPosition(m_body->GetPosition(NWU));

        // Initialization of the parent class.
        FrPrePhysicsItem::Initialize();
        m_chronoPhysicsItem->SetupInitial();

    }

    void FrHydroMesh::Compute(double time) {

        m_clippedMesh.clear();
        m_clippedMesh = m_initMesh;

        // Adjust the position of the clipped mesh according to the position of the body
        UpdateMeshPositionInWorld();

        // Application of the mesh clipper on the updated init mesh to obtain the clipped mesh
        m_clipper->Apply(&m_clippedMesh);

    }

    void FrHydroMesh::UpdateMeshPositionInWorld() {

        // This function transports the mesh from the mesh frame to the body frame, then applies the rotation of mesh
        // in the world frame. Iterating on vertices to get their place wrt to plane.

        // Loop over the vertices.
        for (auto vh : m_clippedMesh.vertices()){

            // From the mesh frame to the body frame.
            m_clippedMesh.point(vh) = GetNodePositionInBody(m_clippedMesh.point(vh));

            auto NodeInBody = mesh::OpenMeshPointToVector3d<Position>(m_clippedMesh.point(vh));

            // Rotation from the body frame to the world frame (just the rotation and the vertical translation, not the horizontal translation of the mesh at the good position in the world mesh).
            // The horizontal translation is not done to avoid numerical errors.
            auto NodeInWorld = m_body->ProjectVectorInWorld<Position>(NodeInBody, NWU);

            // Vertical translation.
            NodeInWorld[2] += m_body->GetPosition(NWU)[2];

            m_clippedMesh.point(vh) = mesh::Vector3dToOpenMeshPoint(NodeInWorld);

        }

    }

    VectorT<double, 3> FrHydroMesh::GetNodePositionInBody(VectorT<double, 3> point) const {


        // From the mesh frame to the body frame: OmP = ObOm + bRm*OmP.
        auto NodeInMeshFrame = mesh::OpenMeshPointToVector3d<Position>(point);

        // Frame transformation, from mesh frame to body frame
        Position NodeInBodyFrame = m_meshOffset.ProjectVectorFrameInParent(NodeInMeshFrame,NWU) + m_meshOffset.GetPosition(NWU);

        return mesh::Vector3dToOpenMeshPoint(NodeInBodyFrame);

    }

    Position FrHydroMesh::GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc){
        // FIXME : voir si on a besoin de ressortir dans le monde ou dans le corps et faire les transformations nécessaires
        // This function returns the center of buoyancy of the clipped mesh in the world frame.

        // The translation of the body was not done for avoiding numerical errors.
        auto CoBInWorld = m_body->GetPosition(NWU) + mesh::OpenMeshPointToVector3d<Position>(m_clippedMesh.GetCOG());

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