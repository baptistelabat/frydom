//
// Created by frongere on 21/06/17.
//

#include "FrLinearHydrostaticForce.h"

/// >>>>>>>>>>>>>>>>> from refactoring

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

/// <<<<<<<<<<<<<<<<<<<<

namespace frydom {


    FrLinearHydrostaticForce::FrLinearHydrostaticForce() {
        force.SetNull();
        moment.SetNull();
    }

    FrLinearHydrostaticStiffnessMatrix *FrLinearHydrostaticForce::GetStiffnessMatrix() { return &m_stiffnessMatrix; }

    void FrLinearHydrostaticForce::UpdateState() {
        // Getting the differential frame from

        auto eqFrame = dynamic_cast<FrHydroBody*>(Body)->GetEquilibriumFrame();
        auto bodyFrame = Body->GetFrame_REF_to_abs();
        //auto deltaFrame = bodyFrame >> eqFrame->GetInverse();
        auto deltaFrame = eqFrame->GetInverse() * bodyFrame;

        double heave = deltaFrame.GetPos().z();

        // ##CC fixe pour monitorer les variations de heave
        m_eqframe_z = eqFrame->GetPos().z();
        m_body_z = bodyFrame.GetPos().z();
        m_delta_z = heave;
        // ##CC

        // Hydrostatic stiffness torque forces directly computed from cardan angles.
        auto cardan_angles = internal::quat_to_euler(deltaFrame.GetRot(),CARDAN,RAD);

        // Multiplying the stiffness matrix with the state vector (heave, roll, pitch)
        auto values = - (m_stiffnessMatrix * chrono::ChVector<double>(heave,cardan_angles.x(),cardan_angles.y()));

        // Distribution of the resulting force in the different hydrostatic components, according to the state vector
        force.z() = values.x();

        // Cancelling gravity on body FIXME : pb de generalisation de la methode
        force -= Body->GetSystem()->Get_G_acc() * Body->GetMass();

        moment.x() = values.y();
        moment.y() = values.z();
    }

    void FrLinearHydrostaticForce::InitializeLogs() {

        m_log.AddField("eqFrame_z","m","Position of the equilibrium frame in z-direction",&m_eqframe_z);
        m_log.AddField("delta_z","m","Pertubation of the position in the z-direction", &m_delta_z);
        m_log.AddField("body_z","m","Position of the body in ref frame",&m_body_z);

        FrForce::InitializeLogs();
    }

    void FrLinearHydrostaticForce::SetLogPrefix(std::string prefix_name) {
        if (prefix_name=="") {
            m_logPrefix = "Fh_" + FrForce::m_logPrefix;
        } else {
            m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
        }
    }


































    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    void FrLinearHydrostaticForce_::Update(double time) {

        auto bodyFrame = m_body->GetFrameAtCOG(NWU);
        auto deltaFrame = m_equilibriumFrame->GetInverse() * bodyFrame;

        Vector3d<double> state; double temp;
        state[0] = deltaFrame.GetPosition(NWU).z();
        deltaFrame.GetRotation().GetCardanAngles_RADIANS(state[1], state[2], temp, NWU);

        auto forceState = - (m_stiffnessMatrix * state);

        auto worldForce = Force(0., 0., forceState[0]);
        worldForce.z() -= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetMass();
        SetForceInWorldAtCOG( worldForce, NWU);

        auto localTorque = Torque(forceState[1], forceState[2], 0.);
        Torque worldTorque = m_equilibriumFrame->ProjectVectorInParent(localTorque);
        Torque bodyTorque = bodyFrame.ProjectVectorParentInFrame(worldTorque);
    }


}  // end namespace frydom