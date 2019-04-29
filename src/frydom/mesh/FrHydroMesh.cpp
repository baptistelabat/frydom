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
                             bool WNL_or_NL) {
        m_system = system;
        m_meshfilename = meshfile;

        m_body = body;
        m_WNL_or_NL = WNL_or_NL;

        // Initialization by default.
        m_Rotation.SetIdentity();
        m_MeshOffset = Position(0,0,0);

        // m_clipper.
        m_clipper = std::make_unique<mesh::MeshClipper>();

    }

    void FrHydroMesh::Initialize() {

        // This function initializes the hydrostatic force object.

        // Loading the input mesh file.
        m_mesh_init = mesh::FrMesh(m_meshfilename);

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

        // Position and orientation of the mesh frame compared to the body frame.
        m_clipper->SetMeshOffsetRotation(m_MeshOffset, m_Rotation);

        // Initialization of the parent class.
        FrPrePhysicsItem::Initialize();
        m_chronoPhysicsItem->SetupInitial();

    }

    void FrHydroMesh::Compute(double time) {

        // This function computes the nonlinear hydrostatic loads.

        m_clipped_mesh = m_clipper->Apply(m_mesh_init);
    }

    Position FrHydroMesh::GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc){

        // This function returns the center of buoyancy of the clipped mesh in the world frame.

        Position CoBInWorld, CoBPos;
        VectorT<double, 3> CoB = m_clipped_mesh.GetCOG(); // Center of gravity of the immersed part (clipped mesh).
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

    void FrHydroMesh::SetMeshOffsetRotation(const Position& Offset, const mathutils::Matrix33<double>& Rotation) {
        m_MeshOffset = Offset;
        m_Rotation = Rotation;

        // Position and orientation of the mesh frame compared to the body frame.
        m_clipper->SetMeshOffsetRotation(m_MeshOffset, m_Rotation);
    }

    mesh::FrMesh FrHydroMesh::GetClippedMesh() {
        return m_clipped_mesh;
    }

    mesh::FrMesh FrHydroMesh::GetInitialMesh() {
        return m_mesh_init;
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