//
// Created by camille on 20/11/18.
//

#include "FrEquilibriumFrame.h"

namespace frydom {


    // ---------------------------------------------------------------------
    // Equilibrium frame
    // ---------------------------------------------------------------------

    void FrEquilibriumFrame_::SetVelocity(const Velocity& velocity) {
        m_velocity = velocity;
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame_::SetAngularVelocity(const double &angularVelocity) {
        m_angularVelocity = angularVelocity;
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame_::SetBody(FrBody_* body, bool initPos) {
        m_body = body;
        m_initPositionFromBody = initPos;
    }

    void FrEquilibriumFrame_::Initialize() {

        if (m_initPositionFromBody) {
            this->SetPosition(m_body->GetPosition(NWU), NWU);
            double temp1, temp2, psi;
            m_body->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, psi, NWU);
            this->GetRotation().RotZ_RADIANS(psi, NWU);
        }

        if (m_initSpeedFromBody) {
            m_velocity = m_body->GetVelocityInWorld(NWU);
            m_angularVelocity = 0.;
        }

    }

    // -----------------------------------------------------------------------
    // Equilibrium frame with spring damping restoring force
    // -----------------------------------------------------------------------

    FrEqFrameSpringDamping_::FrEqFrameSpringDamping_(FrBody_* body, double T0, double psi, bool initPos)
        : FrEquilibriumFrame_(body, initPos) { this->SetSpringDamping(T0, psi); }

    FrEqFrameSpringDamping_::FrEqFrameSpringDamping_(const Position &pos, const FrRotation_ &rotation,
                                                     FRAME_CONVENTION fc, FrBody_* body, double T0, double psi)
            : FrEquilibriumFrame_(pos, rotation, fc, body) { this->SetSpringDamping(T0, psi); }

    FrEqFrameSpringDamping_::FrEqFrameSpringDamping_(const Position &pos, const FrUnitQuaternion_& quaternion,
                                                     FRAME_CONVENTION fc, FrBody_* body, double T0, double psi)
            : FrEquilibriumFrame_(pos, quaternion, fc, body) { this->SetSpringDamping(T0, psi); }

    FrEqFrameSpringDamping_::FrEqFrameSpringDamping_(const FrFrame_& otherFrame, FrBody_* body, double T0, double psi)
            : FrEquilibriumFrame_(otherFrame, body) { this->SetSpringDamping(T0, psi); }

    void FrEqFrameSpringDamping_::SetSpringDamping(const double T0, const double psi) {

        m_w0 = 2.*M_PI / T0;
        m_psi = psi;

        m_damping = 2. * m_psi * m_w0;
        m_stiffness = m_w0 * m_w0;
    }

    void FrEqFrameSpringDamping_::Update(double time) {

        auto bodyPosition = m_body->GetPosition(NWU);
        auto bodyVelocity = m_body->GetVelocityInWorld(NWU);
        auto position = GetPosition(NWU);

        Force force;
        force = -(bodyPosition - position) * m_stiffness - (bodyVelocity - m_velocity) * m_damping;

        double temp1, temp2;
        double bodyPsi, psi;
        GetRotation().GetCardanAngles_RADIANS(temp1, temp2, psi, NWU);
        m_body->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyPsi, NWU);
        auto bodyAngularVelocity = m_body->GetAngularVelocityInWorld(NWU).GetWz();

        double torque;
        torque = -(bodyPsi - psi) * m_stiffness - (bodyAngularVelocity - m_angularVelocity) * m_damping;

        m_velocity += force * (time - m_prevTime);
        position += m_velocity * (time - m_prevTime);

        m_angularVelocity += torque * (time - m_prevTime);

        this->SetPosition(position, NWU);
        this->GetRotation().RotZ_RADIANS(m_angularVelocity * (time - m_prevTime), NWU);

        m_prevTime = time;
    }

    // ----------------------------------------------------------------
    // Equilibrium frame with updated mean velocity
    // ----------------------------------------------------------------

    FrEqFrameMeanMotion_::FrEqFrameMeanMotion_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc,
                                               FrBody_* body, double timePersistence, double timeStep)
            : FrEquilibriumFrame_(pos, rotation, fc, body) { this->SetRecorders(timePersistence, timeStep); }

    FrEqFrameMeanMotion_::FrEqFrameMeanMotion_(const Position &pos, const FrUnitQuaternion_ &quaternion, FRAME_CONVENTION fc,
                                               FrBody_* body, double timePersistence, double timeStep)
            : FrEquilibriumFrame_(pos, quaternion, fc, body) { this->SetRecorders(timePersistence, timeStep); }

    FrEqFrameMeanMotion_::FrEqFrameMeanMotion_(const FrFrame_ &otherFrame, FrBody_* body, double timePersistence, double timeStep)
            : FrEquilibriumFrame_(otherFrame, body) { this->SetRecorders(timePersistence, timeStep); }

    FrEqFrameMeanMotion_::FrEqFrameMeanMotion_(FrBody_ *body, double timePersistence, double timeStep, bool initPos)
    : FrEquilibriumFrame_(body, initPos) { this->SetRecorders(timePersistence, timeStep); }


    void FrEqFrameMeanMotion_::SetRecorders(double timePersistence, double timeStep) {
        m_TrSpeedRec = std::make_unique<FrTimeRecorder_<Velocity>>(timePersistence, timeStep);
        m_TrSpeedRec->Initialize();
        m_AglSpeedRec = std::make_unique<FrTimeRecorder_<double>>(timePersistence, timeStep);
        m_AglSpeedRec->Initialize();
    }


    void FrEqFrameMeanMotion_::Update(double time) {

        m_TrSpeedRec->Record(time, m_body->GetVelocityInWorld(NWU));
        m_AglSpeedRec->Record(time, m_body->GetAngularVelocityInWorld(NWU).GetWz());

        m_velocity = m_TrSpeedRec->GetMean();
        m_angularVelocity = m_AglSpeedRec->GetMean();

        auto position = GetPosition(NWU);
        position += m_velocity * (time - m_prevTime);

        GetRotation().RotZ_RADIANS(m_angularVelocity * (time - m_prevTime), NWU);
        this->SetPosition(position, NWU);

        m_prevTime = time;
    }

}
