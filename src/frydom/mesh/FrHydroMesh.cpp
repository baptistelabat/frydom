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
#include "FrPlane.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/link/constraint/FrCGeometrical.h"
#include "FrTriangleMeshConnected.h"
#include "frydom/asset/shape/FrTriangleMeshShape.h"
#include "frydom/utils/FrIrrApp.h"

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
            case ClippingSupport::PLANESURFACE: {
//                c_nodeForClippingPlane = m_body->GetSystem()->GetWorldBody()->NewNode();
                Position Tide(0., 0., m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetTidal()->GetHeight(NWU));
//                c_nodeForClippingPlane->SetPositionInBody(Tide, NWU);
//
//                auto plane = std::make_shared<FrCPlane>(c_nodeForClippingPlane);

                c_clippingPlane = std::make_shared<geom::FrPlane>(Tide,Direction(0,0,1),NWU);
                auto clippingSurface = std::make_shared<mesh::FrClippingPlane>(c_clippingPlane);
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

        // Rotating the mesh from the body reference frame to the world reference frame, and then translating
        // it vertically. The resulting mesh horizontal position is kept close to (0.,0.) for the clipping process
        // Rotation
        m_clippedMesh.Rotate(m_body->GetRotation().GetRotationMatrix());

        // Translation
        auto bodyPos = m_body->GetPosition(NWU); bodyPos.GetX() = 0.; bodyPos.GetY() = 0.;
        m_clippedMesh.Translate(mesh::Vector3dToOpenMeshPoint(bodyPos));

        // Set the body position for horizontal correction in the clipping surface
        m_clipper->GetClippingSurface()->SetBodyPosition(m_body->GetPosition(NWU));

        // Update the node vertical position for the clippingplane to the position of the tidal height (mean free surface position) 
        if (m_clippingSupport == ClippingSupport::PLANESURFACE) {
            Position Tide(0., 0., m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetTidal()->GetHeight(NWU));
//            c_nodeForClippingPlane->SetPositionInBody(Tide, NWU);
            c_clippingPlane->SetOrigin(Tide, NWU);
        }

        // Application of the mesh clipper on the updated init mesh to obtain the clipped mesh
        m_clipper->Apply(&m_clippedMesh);

        // The clipped mesh obtained at this point is expressed in the world reference frame, but it's horizontal position
        // does not coincide with the body's and is kept close to (0.,0.).

    }

    mesh::FrMesh &FrHydroMesh::ImportMesh(const std::string &meshFile, FrFrame meshOffset) {

        m_initMesh = mesh::FrMesh(meshFile);
        m_initMesh.Translate(mesh::Vector3dToOpenMeshPoint(meshOffset.GetPosition(NWU)));
        m_initMesh.Rotate(meshOffset.GetRotation().GetRotationMatrix());

        return m_initMesh;
    }

    FrHydroMesh::ClippingSupport FrHydroMesh::GetClippingSupport() const {
        return m_clippingSupport;
    }

  void FrHydroMesh::StepFinalize() {
        FrObject::StepFinalize();
        
        if (m_showAsset) {

            // Remove former asset
            RemoveAssets();

            // Get the clipped mesh and translate it to the horizontal body position
            auto tempMesh = m_clippedMesh;
            auto bodyPos = m_body->GetPosition(NWU); bodyPos.GetZ() = 0.;
            tempMesh.Translate(mesh::Vector3dToOpenMeshPoint(bodyPos));

            // Convert the clipped mesh to FrTriangleMeshConnected and add it to the asset owner
            auto triangleMesh = tempMesh.ConvertToTriangleMeshConnected();
            AddMeshAsset(triangleMesh);

            // Bind and update the new asset
            auto irrApp = GetSystem()->GetIrrApp();
            irrApp->AssetBind(GetChronoPhysicsItem());
            irrApp->AssetUpdate(GetChronoPhysicsItem());

        }
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