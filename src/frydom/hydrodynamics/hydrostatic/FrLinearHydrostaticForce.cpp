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

// >>>>>>>>>>>>>>>>> from refactoring

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroMapper.h"

// <<<<<<<<<<<<<<<<<<<<

namespace frydom {


//    FrLinearHydrostaticForce::FrLinearHydrostaticForce() {
//        force.SetNull();
//        moment.SetNull();
//    }
//
//    FrLinearHydrostaticStiffnessMatrix *FrLinearHydrostaticForce::GetStiffnessMatrix() { return &m_stiffnessMatrix; }
//
//    void FrLinearHydrostaticForce::UpdateState() {
//        // Getting the differential frame from
//
//        auto eqFrame = dynamic_cast<FrHydroBody*>(Body)->GetEquilibriumFrame();
//        auto bodyFrame = Body->GetFrame_REF_to_abs();
//        //auto deltaFrame = bodyFrame >> eqFrame->GetInverse();
//        auto deltaFrame = eqFrame->GetInverse() * bodyFrame;
//
//        double heave = deltaFrame.GetPos().z();
//
//        // ##CC fixe pour monitorer les variations de heave
//        m_eqframe_z = eqFrame->GetPos().z();
//        m_body_z = bodyFrame.GetPos().z();
//        m_delta_z = heave;
//        // ##CC
//
//        // Hydrostatic stiffness torque forces directly computed from cardan angles.
//        auto cardan_angles = internal::quat_to_euler(deltaFrame.GetRot(),CARDAN,RAD);
//
//        // Multiplying the stiffness matrix with the state vector (heave, roll, pitch)
//        auto values = - (m_stiffnessMatrix * chrono::ChVector<double>(heave,cardan_angles.x(),cardan_angles.y()));
//
//        // Distribution of the resulting force in the different hydrostatic components, according to the state vector
//        force.z() = values.x();
//
//        // Cancelling gravity on body FIXME : pb de generalisation de la methode
//        force -= Body->GetSystem()->Get_G_acc() * Body->GetMass();
//
//        moment.x() = values.y();
//        moment.y() = values.z();
//    }
//
//    void FrLinearHydrostaticForce::InitializeLogs() {
//
//        m_log.AddField("eqFrame_z","m","Position of the equilibrium frame in z-direction",&m_eqframe_z);
//        m_log.AddField("delta_z","m","Pertubation of the position in the z-direction", &m_delta_z);
//        m_log.AddField("body_z","m","Position of the body in ref frame",&m_body_z);
//
//        FrForce::InitializeLogs();
//    }
//
//    void FrLinearHydrostaticForce::SetLogPrefix(std::string prefix_name) {
//        if (prefix_name=="") {
//            m_logPrefix = "Fh_" + FrForce::m_logPrefix;
//        } else {
//            m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
//        }
//    }
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    void FrLinearHydrostaticForce_::Initialize() {

        /// This subroutine initializes the hydrostatic force object.

        // Initialization of the parent class.
        FrForce_::Initialize();

        // Equilibrium frame of the body.
        m_equilibriumFrame = m_HDB->GetMapper()->GetEquilibriumFrame(m_body);

        // 3x3 hydrostatic matrix.
        m_stiffnessMatrix.SetData(m_HDB->GetBody(m_body)->GetHydrostaticStiffnessMatrix());
    }

    void FrLinearHydrostaticForce_::Update(double time) {

        /// This subroutine computes the hydrostatic loads.

        // Body frame.
        auto bodyFrame = m_body->GetFrameAtCOG(NWU);

        // Transformation from the body frame to equilibrium frame/
        auto deltaFrame = m_equilibriumFrame->GetInverse() * bodyFrame;

        // Position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
        Vector3d<double> state; double temp;
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

    void FrLinearHydrostaticForce_::StepFinalize() {
        FrForce_::StepFinalize();
    }

    std::shared_ptr<FrLinearHydrostaticForce_>
    make_linear_hydrostatic_force(std::shared_ptr<FrHydroDB_> HDB, std::shared_ptr<FrBody_> body){

        /// This subroutine creates the hydrostatic force object for computing the hydrostatic loads.

        // Construction of the hydrostatic force object from the HDB.
        auto forceHst = std::make_shared<FrLinearHydrostaticForce_>(HDB);

        // Add the hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        return forceHst;
    }

}  // end namespace frydom
