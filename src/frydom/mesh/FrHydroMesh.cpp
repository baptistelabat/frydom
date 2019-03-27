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
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    void FrHydroMesh::Initialize() {

        // This function initializes the hydrostatic force object.

        // Initialization of the parent class.
        FrPrePhysicsItem::Initialize();
        m_chronoPhysicsItem->SetupInitial();

        // Loading the input mesh file.
        m_mesh_init = mesh::FrMesh(m_meshfilename);

    }

    void FrHydroMesh::Update(double time) {

        // This function computes the nonlinear hydrostatic loads.

        // Loading the input mesh file.
        mesh::FrMesh current_mesh = m_mesh_init;

        // Clipper.
        mesh::MeshClipper clipper;

        // Tidal height.
        double TidalHeight = m_system->GetEnvironment()->GetOcean()->GetFreeSurface()->GetTidal()->GetHeight(NWU);

        // Clipping surface.
        if(m_WNL_or_NL = true) { // Incident wave field.

            // Incident free surface.
            FrFreeSurface *FreeSurface = m_system->GetEnvironment()->GetOcean()->GetFreeSurface();

            // Setting the free surface.
            clipper.SetWaveClippingSurface(TidalHeight, FreeSurface);
        }
        else{ // Plane.

            // Setting the free surface.
            clipper.SetPlaneClippingSurface(TidalHeight);
        }

        // Body.
        clipper.SetBody(m_body.get());

        // Position and orientation of the mesh frame compared to the body frame.
        clipper.SetMeshOffsetRotation(m_MeshOffset, m_Rotation);

        // Clipping.
        m_clipped_mesh = clipper(current_mesh);

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

    std::shared_ptr<FrHydroMesh> make_hydro_mesh_nonlinear(FrOffshoreSystem* system, std::shared_ptr<FrBody> body, std::string meshfile){

        // This function creates a hydrodynamic mesh for using in the computation of the nonlinear hydrostatic and/or Froude-Krylov loads.

        auto HydroMesh = std::make_shared<FrHydroMesh>(system,meshfile,body,true);

        system->Add(HydroMesh);

        return HydroMesh;
    }

    std::shared_ptr<FrHydroMesh> make_hydro_mesh_weakly_nonlinear(FrOffshoreSystem* system, std::shared_ptr<FrBody> body, std::string meshfile){

        // This function creates a hydrodynamic mesh for using in the computation of the weakly nonlinear hydrostatic and/or Froude-Krylov loads.

        auto HydroMesh = std::make_shared<FrHydroMesh>(system,meshfile,body,false);

        system->Add(HydroMesh);

        return HydroMesh;
    }

}  // end namespace frydom