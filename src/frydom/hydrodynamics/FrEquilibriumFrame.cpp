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


#include "FrEquilibriumFrame.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/utils/FrRecorder.h"


namespace frydom {

    // ---------------------------------------------------------------------
    // Equilibrium frame
    // ---------------------------------------------------------------------

    FrFrame FrEquilibriumFrame::GetPerturbationFrame() {
        return GetInverse() * m_body->GetFrameAtCOG(NWU);;
    }

    void FrEquilibriumFrame::SetVelocityInWorld(const Velocity& velocity, FRAME_CONVENTION fc) {
        if(IsNED(fc)) internal::SwapFrameConvention(velocity);
        m_velocity = velocity;
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame::SetVelocityInFrame(const Velocity& frameVel) { // TODO : voir a ajouter un FRAME_CONVENTION !!
        auto worldVel = ProjectVectorFrameInParent(frameVel, NWU);
        this->SetVelocityInWorld(worldVel, NWU);
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame::SetAngularVelocityAroundZ(const double &angularVelocity, FRAME_CONVENTION fc) {
        m_angularVelocity = angularVelocity;
        if(IsNED(fc))  { m_angularVelocity = -m_angularVelocity; }
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame::SetBody(FrBody* body, bool initPos) {
        m_body = body;
        m_initPositionFromBody = initPos;
    }

    Velocity FrEquilibriumFrame::GetVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity velocity = m_velocity;
        if (IsNED(fc)) internal::SwapFrameConvention(velocity);
        return velocity;
    }

    Velocity FrEquilibriumFrame::GetVelocityInFrame() const { // TODO : voir a ajouter un FRAME_CONVENTION !!
        return ProjectVectorParentInFrame<Velocity>(m_velocity, NWU);
    }

    Velocity FrEquilibriumFrame::GetPerturbationVelocityInWorld(FRAME_CONVENTION fc) const {
        auto frameVelocity = this->GetVelocityInWorld(fc);
        auto bodyVelocity = m_body->GetCOGVelocityInWorld(fc);
        return bodyVelocity - frameVelocity;
    }

    Velocity FrEquilibriumFrame::GetPerturbationVelocityInFrame() const {
        auto velocityInWorld = this->GetPerturbationVelocityInWorld(NWU);
        return ProjectVectorParentInFrame<Velocity>(velocityInWorld, NWU);
    }

    GeneralizedVelocity FrEquilibriumFrame::GetPerturbationGeneralizedVelocityInWorld(FRAME_CONVENTION fc) const {
        auto velocity = GetPerturbationVelocityInWorld(fc);
        auto angularVelocity = GetAngularPerturbationVelocity(fc);
        return GeneralizedVelocity(velocity, angularVelocity);
    }

    GeneralizedVelocity FrEquilibriumFrame::GetPerturbationGeneralizedVelocityInFrame() const {
        auto velocity = GetPerturbationVelocityInFrame();
        auto angularVelocity = GetAngularPerturbationVelocityInFrame();
        return GeneralizedVelocity(velocity, angularVelocity);
    }

    double FrEquilibriumFrame::GetAngularVelocityAroundZ(FRAME_CONVENTION fc) const {
        double result = m_angularVelocity;
        if (IsNED(fc)) { result = -result; }
        return result;
    }

    AngularVelocity FrEquilibriumFrame::GetAngularVelocity(FRAME_CONVENTION fc) const {
        auto wvel = GetAngularVelocityAroundZ(fc);
        return AngularVelocity(0., 0., wvel);
    }

    AngularVelocity FrEquilibriumFrame::GetAngularPerturbationVelocity(FRAME_CONVENTION fc) const {
        auto frameAngularVelocity = GetAngularVelocity(fc);
        auto bodyAngularVelocity = m_body->GetAngularVelocityInWorld(fc);
        return bodyAngularVelocity - frameAngularVelocity;
    }

    AngularVelocity FrEquilibriumFrame::GetAngularPerturbationVelocityInFrame() const {
        auto worldAngularVelocity = this->GetAngularPerturbationVelocity(NWU);
        return ProjectVectorParentInFrame<AngularVelocity>(worldAngularVelocity, NWU);
    }

    void FrEquilibriumFrame::SetPositionToBodyPosition() {
        this->SetPosition(m_body->GetCOGPositionInWorld(NWU), NWU);
        this->SetRotation(m_body->GetRotation());
        m_initPositionFromBody = false;
    }

    void FrEquilibriumFrame::SetVelocityToBodyVelocity() {
        m_velocity = m_body->GetCOGVelocityInWorld(NWU);
        m_angularVelocity = 0.;
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame::Initialize() {

        if(!m_body) { throw FrException("error : the body is not defined in equilibrium frame"); }

        // Log
        SetPathManager(m_system->GetPathManager());

        if (m_initPositionFromBody) this->SetPositionToBodyPosition();
        if (m_initSpeedFromBody) this->SetVelocityToBodyVelocity();

        m_prevTime = 0.;
    }

    void FrEquilibriumFrame::Compute(double time) {

        if (std::abs(time - m_prevTime) < FLT_EPSILON) {
            return;
        }

        auto dt = time - m_prevTime;
        auto prevPosition = this->GetPosition(NWU);

        if (m_velocity.squaredNorm() > FLT_EPSILON) {
            this->SetPosition(prevPosition + m_velocity * dt, NWU);
        }

        if (std::abs(m_angularVelocity) > FLT_EPSILON) {
            this->RotZ_RADIANS(m_angularVelocity * dt, NWU, true);
        }

        m_prevTime = time;
    }

    void FrEquilibriumFrame::AddFields() {

        if (IsLogged()) {

            // Add the fields to be logged here
            // TODO: A completer
            m_message->AddField<double>("time", "s", "Current time of the simulation",
                                        [this]() { return m_system->GetTime(); });

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("Position","m", fmt::format("Equilibrium frame position in the world reference frame in {}", GetLogFrameConvention()),
                     [this]() {return GetPosition(GetLogFrameConvention());});

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("CardanAngles","rad", fmt::format("Equilibrium frame orientation in the world reference frame in {}", GetLogFrameConvention()),
                     [this]() {double phi, theta, psi; GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, GetLogFrameConvention()); return Vector3d<double>(phi, theta, psi);});

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("VelocityInWorld","m/s", fmt::format("Equilibrium frame velocity in the world reference frame in {}", GetLogFrameConvention()),
                     [this]() {return GetVelocityInWorld(GetLogFrameConvention());});

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("AngularVelocity","rad/s", fmt::format("Equilibrium frame angular velocity in the world reference frame in {}", GetLogFrameConvention()),
                     [this]() {return GetAngularVelocity(GetLogFrameConvention());});


            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("PerturbationPosition","m", fmt::format("Perturbation position between the equilibrium frame and the body frame in the world reference frame in {}", GetLogFrameConvention()),
                     [this]() {return GetPerturbationFrame().GetPosition(GetLogFrameConvention());});

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("PerturbationOrientation","m", fmt::format("Perturbation orientation between the equilibrium frame and the body frame in the world reference frame in {}", GetLogFrameConvention()),
                     [this]() {double phi, theta, psi;
                     GetPerturbationFrame().GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, GetLogFrameConvention());
                     return Vector3d<double>(phi, theta, psi);});
            
        }
    }

    void FrEquilibriumFrame::StepFinalize() {

        FrPhysicsItem::StepFinalize();

        // Serialize and send the message log
        FrObject::SendLog();

    }

    // -----------------------------------------------------------------------
    // Equilibrium frame with spring damping restoring force
    // -----------------------------------------------------------------------

    FrEqFrameSpringDamping::FrEqFrameSpringDamping(FrBody* body, double T0, double psi, bool initPos)
        : FrEquilibriumFrame(body, initPos) { this->SetSpringDamping(T0, psi); }

    FrEqFrameSpringDamping::FrEqFrameSpringDamping(const Position &pos, const FrRotation &rotation,
                                                     FRAME_CONVENTION fc, FrBody* body, double T0, double psi)
            : FrEquilibriumFrame(pos, rotation, fc, body) { this->SetSpringDamping(T0, psi); }

    FrEqFrameSpringDamping::FrEqFrameSpringDamping(const Position &pos, const FrUnitQuaternion& quaternion,
                                                     FRAME_CONVENTION fc, FrBody* body, double T0, double psi)
            : FrEquilibriumFrame(pos, quaternion, fc, body) { this->SetSpringDamping(T0, psi); }

    FrEqFrameSpringDamping::FrEqFrameSpringDamping(const FrFrame& otherFrame, FrBody* body, double T0, double psi)
            : FrEquilibriumFrame(otherFrame, body) { this->SetSpringDamping(T0, psi); }

    void FrEqFrameSpringDamping::SetSpringDamping(const double T0, const double psi) {

        m_w0 = 2.*M_PI / T0;
        m_psi = psi;

        m_damping = 2. * m_psi * m_w0;
        m_stiffness = m_w0 * m_w0;
    }

    void FrEqFrameSpringDamping::Compute(double time) {

        if (std::abs(time - m_prevTime) < FLT_EPSILON) return;

        auto bodyPosition = m_body->GetCOGPositionInWorld(NWU);
        auto bodyVelocity = m_body->GetCOGVelocityInWorld(NWU);
        auto position = GetPosition(NWU);

        Force force;
        force = (bodyPosition - position) * m_stiffness + (bodyVelocity - m_velocity) * m_damping;
        force.GetFz() = 0.;

        double temp1, temp2;
        double bodyPsi, psi;
        GetRotation().GetCardanAngles_RADIANS(temp1, temp2, psi, NWU);
        m_body->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyPsi, NWU);
        auto bodyAngularVelocity = m_body->GetAngularVelocityInWorld(NWU).GetWz();

        double torque;
        torque = (bodyPsi - psi) * m_stiffness + (bodyAngularVelocity - m_angularVelocity) * m_damping;

        m_velocity += force * (time - m_prevTime);
        position += m_velocity * (time - m_prevTime);

        m_angularVelocity += torque * (time - m_prevTime);

        this->SetPosition(position, NWU);
        SetRotation( this->GetRotation().RotZ_RADIANS(m_angularVelocity * (time - m_prevTime), NWU) );

        m_prevTime = time;
    }

    // ----------------------------------------------------------------
    // Equilibrium frame with updated mean velocity
    // ----------------------------------------------------------------

    FrEqFrameMeanMotion::FrEqFrameMeanMotion(const Position &pos, const FrRotation &rotation, FRAME_CONVENTION fc,
                                               FrBody* body, double timePersistence, double timeStep)
            : FrEquilibriumFrame(pos, rotation, fc, body) { this->SetRecorders(timePersistence, timeStep); }

    FrEqFrameMeanMotion::FrEqFrameMeanMotion(const Position &pos, const FrUnitQuaternion &quaternion, FRAME_CONVENTION fc,
                                               FrBody* body, double timePersistence, double timeStep)
            : FrEquilibriumFrame(pos, quaternion, fc, body) { this->SetRecorders(timePersistence, timeStep); }

    FrEqFrameMeanMotion::FrEqFrameMeanMotion(const FrFrame &otherFrame, FrBody* body, double timePersistence, double timeStep)
            : FrEquilibriumFrame(otherFrame, body) { this->SetRecorders(timePersistence, timeStep); }

    FrEqFrameMeanMotion::FrEqFrameMeanMotion(FrBody *body, double timePersistence, double timeStep, bool initPos)
    : FrEquilibriumFrame(body, initPos) { this->SetRecorders(timePersistence, timeStep); }


    void FrEqFrameMeanMotion::SetRecorders(double timePersistence, double timeStep) {
        m_TrSpeedRec = std::make_unique<FrTimeRecorder<Velocity>>(timePersistence, timeStep);
        m_TrSpeedRec->Initialize();
        m_AglSpeedRec = std::make_unique<FrTimeRecorder<double>>(timePersistence, timeStep);
        m_AglSpeedRec->Initialize();
    }

    void FrEqFrameMeanMotion::SetPositionCorrection(double timePersistence, double timeStep,
                                                     double posCoeff, double angleCoeff) {
        m_ErrPositionRec = std::make_unique<FrTimeRecorder<Position>>(timePersistence, timeStep);
        m_ErrPositionRec->Initialize();
        m_ErrAngleRec = std::make_unique<FrTimeRecorder<double>>(timePersistence, timeStep);
        m_ErrAngleRec->Initialize();
        m_errPosCoeff = posCoeff;
        m_errAngleCoeff = angleCoeff;
    }

    void FrEqFrameMeanMotion::Compute(double time) {

        if (std::abs(time - m_prevTime) < FLT_EPSILON) return;

        m_TrSpeedRec->Record(time, m_body->GetCOGVelocityInWorld(NWU));
        m_AglSpeedRec->Record(time, m_body->GetAngularVelocityInWorld(NWU).GetWz());

        m_velocity = m_TrSpeedRec->GetMean();
        m_angularVelocity = m_AglSpeedRec->GetMean();

        auto position = GetPosition(NWU);
        position += m_velocity * (time - m_prevTime);

        auto angle = m_angularVelocity * (time - m_prevTime);

        if (m_ErrPositionRec and m_ErrAngleRec) {

            m_ErrPositionRec->Record(time, m_body->GetCOGPositionInWorld(NWU) - GetPosition(NWU));
            auto errMeanPosition = m_ErrPositionRec->GetMean();
            position += errMeanPosition * m_errPosCoeff;

            double temp1, temp2, bodyAngle, frameAngle;
            m_body->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyAngle, NWU);
            this->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, frameAngle, NWU);
            m_ErrAngleRec->Record(time, bodyAngle - frameAngle);
            auto errMeanAngle = m_ErrAngleRec->GetMean();
            angle += errMeanAngle * m_errAngleCoeff;
        }

        this->SetPosition(position, NWU);
        SetRotation( GetRotation().RotZ_RADIANS(angle, NWU));

        m_prevTime = time;
    }

}  // end namespace frydom
