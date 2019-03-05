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


#include "FrLinearHydrostaticForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"



namespace frydom {

    void FrLinearHydrostaticForce::Initialize() {

        /// This subroutine initializes the hydrostatic force object.

        // Initialization of the parent class.
        FrForce::Initialize();

        // Equilibrium frame of the body.
        m_equilibriumFrame = m_HDB->GetMapper()->GetEquilibriumFrame(m_body);

        // 3x3 hydrostatic matrix.
        m_stiffnessMatrix.SetData(m_HDB->GetBody(m_body)->GetHydrostaticStiffnessMatrix());
    }

    void FrLinearHydrostaticForce::Update(double time) {

        /// This subroutine computes the linear hydrostatic loads.

        // Body frame.
        auto bodyFrame = m_body->GetFrameAtCOG(NWU);

        // Transformation from the body frame to equilibrium frame/
        auto deltaFrame = m_equilibriumFrame->GetInverse() * bodyFrame;

        // Position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
        mathutils::Vector3d<double> state; double temp;
        state[0] = deltaFrame.GetPosition(NWU).z();

        // Angular position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
        deltaFrame.GetRotation().GetCardanAngles_RADIANS(state[1], state[2], temp, NWU);

        // Fh = -Kh*X. in the equilibrium frame: only heave, roll and pitch are considered here.
        auto forceState = - (m_stiffnessMatrix * state); // m_stiffnessMatrix is a 3x3 matrix.

        // Linear hydrostatic force: assumed in the world frame.
        auto worldForce = Force(0., 0., forceState[0]); // Only the heave component is used from forceState, so the first one.
        worldForce.z() += m_body->GetSystem()->GetGravityAcceleration() * m_body->GetMass(); // WARNING: It is assumed that the displacement is equal to the mass, which can be false.
        SetForceInWorldAtCOG( worldForce, NWU);

        // Linear hydrostatic torque: assumed in the body frame/
        auto localTorque = Torque(forceState[1], forceState[2], 0.);
        SetTorqueInBodyAtCOG(localTorque, NWU);
    }

    void FrLinearHydrostaticForce::StepFinalize() {
        FrForce::StepFinalize();
    }

    std::shared_ptr<FrLinearHydrostaticForce>
    make_linear_hydrostatic_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body){

        /// This subroutine creates the linear hydrostatic force object for computing the linear hydrostatic loads.

        // Construction of the hydrostatic force object from the HDB.
        auto forceHst = std::make_shared<FrLinearHydrostaticForce>(HDB);

        // Add the hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        return forceHst;
    }

}  // end namespace frydom
