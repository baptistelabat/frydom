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

#include "FrWeaklyNonlinearHydrostaticForce.h"

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroMapper.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/mesh/FrHydrostaticsProperties.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    void FrWeaklyNonlinearHydrostaticForce::Initialize() {

        // This function initializes the hydrostatic force object.

        // Initialization of the parent class.
        FrForce::Initialize();

        // Loading the input mesh file.
        m_mesh_init = mesh::FrMesh(meshfilename);

    }

    void FrWeaklyNonlinearHydrostaticForce::InitializeLog(){

        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("Center of buoyancy","m", fmt::format("Center of buoyancy in world reference frame in {}", c_logFrameConvention),
                 [this]() {return GetCenterOfBuoyancyInBody(c_logFrameConvention);});

        FrForce::InitializeLog();

    }

    void FrWeaklyNonlinearHydrostaticForce::Update(double time) {

        // This function computes the weakly nonlinear hydrostatic loads.

        // To know which force object (HS or FK) clips the mesh, to avoid doing that twice.
        if(m_body->GetHSFK() == 0){
            m_body->SetHSFK(1); // 1 for hydrostatic, -1 for Froude-Krylov.
        }

        if(m_body->GetHSFK() == 1) { // If 1, then the mesh is clipped here and the Froude-Krylov force object will use it too.

            // Loading the input mesh file.
            mesh::FrMesh current_mesh = m_mesh_init;

            // Clipper.
            mesh::MeshClipper clipper;

            // Tidal height.
            double TidalHeight = m_system->GetEnvironment()->GetOcean()->GetFreeSurface()->GetTidal()->GetHeight(NWU);

            // Clipping surface.
            clipper.SetPlaneClippingSurface(TidalHeight);

            // Body.
            clipper.SetBody(m_body);

            // Position and orientation of the mesh frame compared to the body frame.
            clipper.SetMeshOffsetRotation(m_MeshOffset, m_Rotation);

            // Clipping.
            m_clipped_mesh = clipper(current_mesh);

            // Storage of the clipped for using in the computation of the weakly or fully nonlinear Froude-Krylov loads.
            m_body->SetClippedMesh(&m_clipped_mesh);
        }
        else{ // If -1, then the mesh is clipped by the Froude-Krylov force object and is also used here.
            m_clipped_mesh = *m_body->GetClippedMesh();
        }

        // Computation of the hydrostatic force.
        NonlinearHydrostatics NLhydrostatics(m_HDB->GetWaterDensity(),m_HDB->GetGravityAcc()); // Creation of the NonlinearHydrostatics structure.
        NLhydrostatics.CalcPressureIntegration(m_clipped_mesh);

        // Setting the weakly nonlinear loads in world.
        Force force = NLhydrostatics.GetWeaklyNonlinearForce();
        m_CoBInWorld = m_body->GetPosition(NWU) + NLhydrostatics.GetCenterOfBuoyancy(); // The translation of the body was not done for avoiding numerical erros.

        this->SetForceInWorldAtPointInWorld(force,m_CoBInWorld,NWU); // The torque is computed from the hydrostatic force and the center of buoyancy.

    }

    void FrWeaklyNonlinearHydrostaticForce::StepFinalize() {
        FrForce::StepFinalize();

        // Writting the clipped mesh in an output file.
//        m_clipped_mesh.Write("Mesh_clipped.obj");

    }

    Position FrWeaklyNonlinearHydrostaticForce::GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc){

        // This function gives the center of buoyancy in the body frame.

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(m_CoBInWorld);
        return m_CoBInWorld;
    }

    std::shared_ptr<FrWeaklyNonlinearHydrostaticForce>
    make_weakly_nonlinear_hydrostatic_force(FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::string meshfile){

        // This function creates the weakly nonlinear hydrostatic force object for computing the weakly nonlinear hydrostatic loads.

        // Construction of the hydrostatic force object from the HDB.
        auto forceHst = std::make_shared<FrWeaklyNonlinearHydrostaticForce>(system,HDB,meshfile);

        // Add the hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        return forceHst;
    }

}  // end namespace frydom