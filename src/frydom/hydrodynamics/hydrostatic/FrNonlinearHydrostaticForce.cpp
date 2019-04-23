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

#include "FrNonlinearHydrostaticForce.h"

#include "frydom/core/math/FrVector.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/mesh/FrHydrostaticsProperties.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    void FrNonlinearHydrostaticForce::Initialize() {

        // This function initializes the hydrostatic force object.

        // Initialization of the parent class.
        FrForce::Initialize();

    }

    void FrNonlinearHydrostaticForce::InitializeLog(){

        // This function initializes the logger for the nonlinear hydrostatic loads by giving the position of the center of buoyancy in the body frame.

        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("CenterOfBuoyancyInBody","m", fmt::format("Center of buoyancy in world reference frame in {}", c_logFrameConvention),
                 [this]() {return GetCenterOfBuoyancyInBody(c_logFrameConvention);});

        FrForce::InitializeLog();

    }

    void FrNonlinearHydrostaticForce::Compute(double time) {

        // This function computes the nonlinear hydrostatic loads.

        // Clipped mesh.
        m_clipped_mesh = m_hydro_mesh->GetClippedMesh();

        // Computation of the hydrostatic force.
        NonlinearHydrostatics NLhydrostatics(m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                             m_body->GetSystem()->GetGravityAcceleration()); // Creation of the NonlinearHydrostatics structure.
        NLhydrostatics.CalcPressureIntegration(m_clipped_mesh);

        // Setting the nonlinear hydrostatic loads in world at the CoB in world.
        Force force = NLhydrostatics.GetNonlinearForce();

        m_CoBInWorld = m_body->GetPosition(NWU) +
                       NLhydrostatics.GetCenterOfBuoyancy(); // The translation of the body was not done for avoiding numerical errors.

        this->SetForceInWorldAtPointInWorld(force, m_CoBInWorld,
                                            NWU); // The torque is computed from the hydrostatic force and the center of buoyancy.

    }

    void FrNonlinearHydrostaticForce::StepFinalize() {
        FrForce::StepFinalize();

        // Writing the clipped mesh in an output file.
//        m_clipped_mesh.Write("Mesh_clipped_Hydrostatics.obj");
//        std::exit(0);

    }

    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc){

        // This function gives the center of buoyancy in the world frame.

        Position CoBInBody = m_body->GetPointPositionInBody(m_CoBInWorld,fc);

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(CoBInBody);
        return CoBInBody;
    }

    std::shared_ptr<FrNonlinearHydrostaticForce>
    make_nonlinear_hydrostatic_force(std::shared_ptr<FrBody> body, std::shared_ptr<FrHydroMesh> HydroMesh){

        // This function creates a (fully or weakly) nonlinear hydrostatic force object.

        // Construction of the (fully or weakly) nonlinear hydrostatic force object.
        auto forceHst = std::make_shared<FrNonlinearHydrostaticForce>(HydroMesh);

        // Add the (fully or weakly) nonlinear hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        return forceHst;
    }

}  // end namespace frydom