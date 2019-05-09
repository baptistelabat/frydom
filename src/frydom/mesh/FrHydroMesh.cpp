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

    FrHydroMesh::FrHydroMesh(FrOffshoreSystem *system, const std::shared_ptr<FrBody>& body, bool WNL_or_NL)
            : m_system(system), m_body(body), m_WNL_or_NL(WNL_or_NL) {
        // m_clipper
        m_clipper = std::make_unique<mesh::FrMeshClipper>();

    }

    FrHydroMesh::FrHydroMesh(FrOffshoreSystem *system, const std::shared_ptr<FrBody>& body, const std::string& meshFile,
                             FrFrame meshOffset, bool WNL_or_NL)
                             : m_system(system), m_body(body), m_WNL_or_NL(WNL_or_NL) {

        // Import and transform the initial mesh, into the body reference frame
        ImportMesh(meshFile, meshOffset);

        // m_clipper
        m_clipper = std::make_unique<mesh::FrMeshClipper>();

    }

    void FrHydroMesh::Initialize() {

        // This function initializes the hydrostatic force object.

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

//    void FrHydroMesh::SetMeshOffset(FrFrame meshOffset) {
//        m_meshOffset = meshOffset;
//    }
//
//    FrFrame FrHydroMesh::GetMeshOffset() const {
//        return m_meshOffset;
//    }

    mesh::FrMesh& FrHydroMesh::GetClippedMesh() {
        return m_clippedMesh;
    }

    mesh::FrMesh& FrHydroMesh::GetInitialMesh() {
        return m_initMesh;
    }

    void FrHydroMesh::Compute(double time) {

        m_clippedMesh.clear();
        m_clippedMesh = m_initMesh;

        // This function rotate the mesh from the body reference frame to the world reference frame, and then translate
        // it vertically. The resulting mesh horizontal position is kept close to (0.,0.) for the clipping process
        UpdateMeshFrame();

        // Application of the mesh clipper on the updated init mesh to obtain the clipped mesh
        m_clipper->Apply(&m_clippedMesh);

        // The clipped mesh obtained at this point is expressed in the world reference frame, but it's horizontal position
        // does not coincide with the body's and is kept close to (0.,0.).

    }

    void FrHydroMesh::UpdateMeshFrame() {

        // This function rotate the mesh from the body reference frame to the world reference frame, and then translate
        // it vertically. The resulting mesh horizontal position is kept close to (0.,0.) for the clipping process

        // Loop over the vertices.
        for (auto vh : m_clippedMesh.vertices()){

//            // From the mesh frame to the body frame.
//            m_clippedMesh.point(vh) = GetMeshPointPositionInBody(m_clippedMesh.point(vh));

            auto NodeInBody = mesh::OpenMeshPointToVector3d<Position>(m_clippedMesh.point(vh));

            // Rotation from the body frame to the world frame (just the rotation and the vertical translation, not the horizontal translation of the mesh at the good position in the world mesh).
            // The horizontal translation is not done to avoid numerical errors.
            auto NodeInWorld = m_body->ProjectVectorInWorld<Position>(NodeInBody, NWU);

            // Vertical translation.
            NodeInWorld[2] += m_body->GetPosition(NWU)[2];

            m_clippedMesh.point(vh) = mesh::Vector3dToOpenMeshPoint(NodeInWorld);

        }

    }

    VectorT<double, 3> FrHydroMesh::GetMeshPointPositionInBody(VectorT<double, 3> point) const {


        // From the mesh frame to the body frame: OmP = ObOm + bRm*OmP.
        auto NodeInMeshFrame = mesh::OpenMeshPointToVector3d<Position>(point);

        // Frame transformation, from mesh frame to body frame
        Position NodeInBodyFrame = m_meshOffset.ProjectVectorFrameInParent(NodeInMeshFrame,NWU) + m_meshOffset.GetPosition(NWU);

        return mesh::Vector3dToOpenMeshPoint(NodeInBodyFrame);

    }

    mesh::FrMesh &FrHydroMesh::ImportMesh(const std::string &meshFile, FrFrame meshOffset) {

        m_initMesh = mesh::FrMesh(meshFile);
        m_initMesh.Translate(mesh::Vector3dToOpenMeshPoint(meshOffset.GetPosition(NWU)));
        double phi, theta, psi;
        meshOffset.GetRotation().GetCardanAngles_RADIANS(phi,theta,psi,NWU);
        m_initMesh.Rotate(phi, theta, psi);

        return m_initMesh;
    }

    std::shared_ptr<FrHydroMesh> make_hydro_mesh(const std::shared_ptr<FrBody>& body, bool NL_or_WNL) {

        auto hydroMesh = std::make_shared<FrHydroMesh>(body->GetSystem(), body, NL_or_WNL);

        body->GetSystem()->Add(hydroMesh);

        return hydroMesh;

    }

    std::shared_ptr<FrHydroMesh> make_hydro_mesh(const std::shared_ptr<FrBody>& body, const std::string& meshFile,
            FrFrame meshOffset, bool NL_or_WNL) {

        auto hydroMesh = std::make_shared<FrHydroMesh>(body->GetSystem(), body, meshFile, meshOffset, NL_or_WNL);

        body->GetSystem()->Add(hydroMesh);

        return hydroMesh;

    }

//    std::shared_ptr<FrHydroMesh> make_hydro_mesh_nonlinear(const std::shared_ptr<FrBody>& body, const std::string& meshfile){
//
//        // This function creates a hydrodynamic mesh for using in the computation of the nonlinear hydrostatic and/or Froude-Krylov loads.
//
//        auto HydroMesh = std::make_shared<FrHydroMesh>(body->GetSystem(),meshfile,body,true);
//
//        body->GetSystem()->Add(HydroMesh);
//
//        return HydroMesh;
//    }
//
//    std::shared_ptr<FrHydroMesh> make_hydro_mesh_weakly_nonlinear(const std::shared_ptr<FrBody>& body, const std::string& meshfile){
//
//        // This function creates a hydrodynamic mesh for using in the computation of the weakly nonlinear hydrostatic and/or Froude-Krylov loads.
//
//        auto HydroMesh = std::make_shared<FrHydroMesh>(body->GetSystem(),meshfile,body,false);
//
//        body->GetSystem()->Add(HydroMesh);
//
//        return HydroMesh;
//    }

}  // end namespace frydom