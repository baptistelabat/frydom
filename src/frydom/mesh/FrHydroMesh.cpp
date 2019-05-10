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

#include "FrMeshClipper.h"

#include "frydom/core/body/FrBody.h"

namespace frydom {

    FrHydroMesh::FrHydroMesh(FrOffshoreSystem *system, const std::shared_ptr<FrBody>& body, FrHydroMesh::ClippingSupport support)
            : m_system(system), m_body(body), m_clippingSupport(support) {
        // m_clipper
        m_clipper = std::make_unique<mesh::FrMeshClipper>();

        m_clippedMesh = mesh::FrMesh();

    }

    FrHydroMesh::FrHydroMesh(FrOffshoreSystem *system, const std::shared_ptr<FrBody>& body, const std::string& meshFile,
                             FrFrame meshOffset, FrHydroMesh::ClippingSupport support)
                             : m_system(system), m_body(body), m_clippingSupport(support) {

        // Import and transform the initial mesh, into the body reference frame
        ImportMesh(meshFile, meshOffset);

        // m_clipper
        m_clipper = std::make_unique<mesh::FrMeshClipper>();

        m_clippedMesh = mesh::FrMesh();

    }

    void FrHydroMesh::Initialize() {

        // This function initializes the hydrostatic force object.


        // Clipping surface.
        switch (m_clippingSupport) {
            case ClippingSupport::PLANSURFACE: {
                auto clippingSurface = std::make_shared<mesh::FrClippingPlane>(
                        m_system->GetEnvironment()->GetOcean()->GetFreeSurface());
                m_clipper->SetClippingSurface(clippingSurface);
                break;
            }
            case ClippingSupport::WAVESURFACE: {
                auto clippingSurface = std::make_shared<mesh::FrClippingWaveSurface>(
                        m_system->GetEnvironment()->GetOcean()->GetFreeSurface());
                m_clipper->SetClippingSurface(clippingSurface);
                break;
            }
        }

        // Initialization of the parent class.
        FrPrePhysicsItem::Initialize();
        m_chronoPhysicsItem->SetupInitial();

    }

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
        //FIXME: faire ça ici, plutôt que dans la methode
        UpdateMeshFrame();

        //FIXME: refiler la position du corps a clippingSurface pour appliquer la correction

        // Set the body position for horizontal correction in the clipping surface
        m_clipper->GetClippingSurface()->SetBodyPosition(m_body->GetPosition(NWU));

        // Application of the mesh clipper on the updated init mesh to obtain the clipped mesh
        m_clipper->Apply(&m_clippedMesh);

        // The clipped mesh obtained at this point is expressed in the world reference frame, but it's horizontal position
        // does not coincide with the body's and is kept close to (0.,0.).

    }

    void FrHydroMesh::UpdateMeshFrame() {

        //FIXME : utiliser plutôt les méthodes de FRMesh pour rotation et translation

        // This function rotate the mesh from the body reference frame to the world reference frame, and then translate
        // it vertically. The resulting mesh horizontal position is kept close to (0.,0.) for the clipping process

//        // Rotation
//        double phi, theta, psi;
//        m_body->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
//        m_clippedMesh.Rotate(phi, theta, psi);
//
//        // Translation
//        auto bodyPos = m_body->GetPosition(NWU); bodyPos.GetX() = 0.; bodyPos.GetY() = 0.;
//        m_clippedMesh.Translate(mesh::Vector3dToOpenMeshPoint(bodyPos));


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

    mesh::FrMesh &FrHydroMesh::ImportMesh(const std::string &meshFile, FrFrame meshOffset) {

        m_initMesh = mesh::FrMesh(meshFile);
        m_initMesh.Translate(mesh::Vector3dToOpenMeshPoint(meshOffset.GetPosition(NWU)));
        double phi, theta, psi;
        meshOffset.GetRotation().GetCardanAngles_RADIANS(phi,theta,psi,NWU);
        m_initMesh.Rotate(phi, theta, psi);

        return m_initMesh;
    }

    std::shared_ptr<FrHydroMesh> make_hydro_mesh(const std::shared_ptr<FrBody>& body, FrHydroMesh::ClippingSupport support) {

        auto hydroMesh = std::make_shared<FrHydroMesh>(body->GetSystem(), body, support);

        body->GetSystem()->Add(hydroMesh);

        return hydroMesh;

    }

    std::shared_ptr<FrHydroMesh> make_hydro_mesh(const std::shared_ptr<FrBody>& body, const std::string& meshFile,
            FrFrame meshOffset, FrHydroMesh::ClippingSupport support) {

        auto hydroMesh = std::make_shared<FrHydroMesh>(body->GetSystem(), body, meshFile, meshOffset, support);

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