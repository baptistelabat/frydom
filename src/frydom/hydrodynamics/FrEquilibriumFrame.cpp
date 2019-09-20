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
    template<typename OffshoreSystemType>
    FrFrame FrEquilibriumFrame<OffshoreSystemType>::GetPerturbationFrame() {
      return GetInverse() * m_body->GetFrameAtCOG(NWU);;
    }

    template<typename OffshoreSystemType>
    void FrEquilibriumFrame<OffshoreSystemType>::SetVelocityInWorld(const Velocity &velocity, FRAME_CONVENTION fc) {
      if (IsNED(fc)) internal::SwapFrameConvention(velocity);
      m_velocity = velocity;
      m_initSpeedFromBody = false;
    }

    template<typename OffshoreSystemType>
    void
    FrEquilibriumFrame<OffshoreSystemType>::SetVelocityInFrame(
        const Velocity &frameVel) { // TODO : voir a ajouter un FRAME_CONVENTION !!
      auto worldVel = ProjectVectorFrameInParent(frameVel, NWU);
      this->SetVelocityInWorld(worldVel, NWU);
      m_initSpeedFromBody = false;
    }

    template<typename OffshoreSystemType>
    void FrEquilibriumFrame<OffshoreSystemType>::SetAngularVelocityAroundZ(const double &angularVelocity,
                                                                           FRAME_CONVENTION fc) {
      m_angularVelocity = angularVelocity;
      if (IsNED(fc)) { m_angularVelocity = -m_angularVelocity; }
      m_initSpeedFromBody = false;
    }

    template<typename OffshoreSystemType>
    void FrEquilibriumFrame<OffshoreSystemType>::SetBody(FrBody<OffshoreSystemType> *body, bool initPos) {
      m_body = body;
      m_initPositionFromBody = initPos;
    }

    template<typename OffshoreSystemType>
    Velocity FrEquilibriumFrame<OffshoreSystemType>::GetVelocityInWorld(FRAME_CONVENTION fc) const {
      Velocity velocity = m_velocity;
      if (IsNED(fc)) internal::SwapFrameConvention(velocity);
      return velocity;
    }

    template<typename OffshoreSystemType>
    Velocity
    FrEquilibriumFrame<OffshoreSystemType>::GetVelocityInFrame() const { // TODO : voir a ajouter un FRAME_CONVENTION !!
      return ProjectVectorParentInFrame<Velocity>(m_velocity, NWU);
    }

    template<typename OffshoreSystemType>
    Velocity FrEquilibriumFrame<OffshoreSystemType>::GetPerturbationVelocityInWorld(FRAME_CONVENTION fc) const {
      auto frameVelocity = this->GetVelocityInWorld(fc);
      auto bodyVelocity = m_body->GetCOGLinearVelocityInWorld(fc);
      return bodyVelocity - frameVelocity;
    }

    template<typename OffshoreSystemType>
    Velocity FrEquilibriumFrame<OffshoreSystemType>::GetPerturbationVelocityInFrame() const {
      auto velocityInWorld = this->GetPerturbationVelocityInWorld(NWU);
      return ProjectVectorParentInFrame<Velocity>(velocityInWorld, NWU);
    }

    template<typename OffshoreSystemType>
    GeneralizedVelocity
    FrEquilibriumFrame<OffshoreSystemType>::GetPerturbationGeneralizedVelocityInWorld(FRAME_CONVENTION fc) const {
      auto velocity = GetPerturbationVelocityInWorld(fc);
      auto angularVelocity = GetAngularPerturbationVelocity(fc);
      return GeneralizedVelocity(velocity, angularVelocity);
    }

    template<typename OffshoreSystemType>
    GeneralizedVelocity FrEquilibriumFrame<OffshoreSystemType>::GetPerturbationGeneralizedVelocityInFrame() const {
      auto velocity = GetPerturbationVelocityInFrame();
      auto angularVelocity = GetAngularPerturbationVelocityInFrame();
      return GeneralizedVelocity(velocity, angularVelocity);
    }

    template<typename OffshoreSystemType>
    double FrEquilibriumFrame<OffshoreSystemType>::GetAngularVelocityAroundZ(FRAME_CONVENTION fc) const {
      double result = m_angularVelocity;
      if (IsNED(fc)) { result = -result; }
      return result;
    }

    template<typename OffshoreSystemType>
    AngularVelocity FrEquilibriumFrame<OffshoreSystemType>::GetAngularVelocity(FRAME_CONVENTION fc) const {
      auto wvel = GetAngularVelocityAroundZ(fc);
      return AngularVelocity(0., 0., wvel);
    }

    template<typename OffshoreSystemType>
    AngularVelocity FrEquilibriumFrame<OffshoreSystemType>::GetAngularPerturbationVelocity(FRAME_CONVENTION fc) const {
      auto frameAngularVelocity = GetAngularVelocity(fc);
      auto bodyAngularVelocity = m_body->GetAngularVelocityInWorld(fc);
      return bodyAngularVelocity - frameAngularVelocity;
    }

    template<typename OffshoreSystemType>
    AngularVelocity FrEquilibriumFrame<OffshoreSystemType>::GetAngularPerturbationVelocityInFrame() const {
      auto worldAngularVelocity = this->GetAngularPerturbationVelocity(NWU);
      return ProjectVectorParentInFrame<AngularVelocity>(worldAngularVelocity, NWU);
    }

    template<typename OffshoreSystemType>
    void FrEquilibriumFrame<OffshoreSystemType>::SetPositionToBodyPosition() {
      this->SetPosition(m_body->GetCOGPositionInWorld(NWU), NWU);
      this->SetRotation(m_body->GetRotation());
      m_initPositionFromBody = false;
    }

    template<typename OffshoreSystemType>
    void FrEquilibriumFrame<OffshoreSystemType>::SetVelocityToBodyVelocity() {
      m_velocity = m_body->GetCOGLinearVelocityInWorld(NWU);
      m_angularVelocity = 0.;
      m_initSpeedFromBody = false;
    }

    template<typename OffshoreSystemType>
    void FrEquilibriumFrame<OffshoreSystemType>::Initialize() {

      if (!m_body) { throw FrException("error : the body is not defined in equilibrium frame"); }

      if (m_initPositionFromBody) this->SetPositionToBodyPosition();
      if (m_initSpeedFromBody) this->SetVelocityToBodyVelocity();

      m_prevTime = 0.;
    }

    template<typename OffshoreSystemType>
    void FrEquilibriumFrame<OffshoreSystemType>::Compute(double time) {

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

    template<typename OffshoreSystemType>
    void FrEquilibriumFrame<OffshoreSystemType>::AddFields() {

      if (this->IsLogged()) {

        // Add the fields to be logged here
        // TODO: A completer
        this->m_message->template AddField<double>("time", "s", "Current time of the simulation",
                                                   [this]() { return this->m_system->GetTime(); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("Position", "m",
             fmt::format("Equilibrium frame position in the world reference frame in {}",
                         this->GetLogFrameConvention()),
             [this]() { return GetPosition(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("CardanAngles", "rad",
             fmt::format("Equilibrium frame orientation in the world reference frame in {}",
                         this->GetLogFrameConvention()),
             [this]() {
               double phi, theta, psi;
               GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, this->GetLogFrameConvention());
               return Vector3d<double>(phi, theta, psi);
             });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("VelocityInWorld", "m/s",
             fmt::format("Equilibrium frame velocity in the world reference frame in {}",
                         this->GetLogFrameConvention()),
             [this]() { return GetVelocityInWorld(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("AngularVelocity", "rad/s",
             fmt::format("Equilibrium frame angular velocity in the world reference frame in {}",
                         this->GetLogFrameConvention()),
             [this]() { return GetAngularVelocity(this->GetLogFrameConvention()); });


        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("PerturbationPosition", "m", fmt::format(
                "Perturbation position between the equilibrium frame and the body frame in the world reference frame in {}",
                this->GetLogFrameConvention()),
             [this]() { return GetPerturbationFrame().GetPosition(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("PerturbationOrientation", "m", fmt::format(
                "Perturbation orientation between the equilibrium frame and the body frame in the world reference frame in {}",
                this->GetLogFrameConvention()),
             [this]() {
               double phi, theta, psi;
               GetPerturbationFrame().GetRotation().GetCardanAngles_RADIANS(phi, theta, psi,
                                                                            this->GetLogFrameConvention());
               return Vector3d<double>(phi, theta, psi);
             });

      }
    }

    template<typename OffshoreSystemType>
    void FrEquilibriumFrame<OffshoreSystemType>::StepFinalize() {

      FrPhysicsItem<OffshoreSystemType>::StepFinalize();

      // Serialize and send the message log
      FrObject<OffshoreSystemType>::SendLog();

    }

    // -----------------------------------------------------------------------
    // Equilibrium frame with spring damping restoring force
    // -----------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrEqFrameSpringDamping<OffshoreSystemType>::FrEqFrameSpringDamping(FrBody<OffshoreSystemType> *body, double T0,
                                                                       double psi, bool initPos)
        : FrEquilibriumFrame<OffshoreSystemType>(body, initPos) { this->SetSpringDamping(T0, psi); }

    template<typename OffshoreSystemType>
    FrEqFrameSpringDamping<OffshoreSystemType>::FrEqFrameSpringDamping(const Position &pos, const FrRotation &rotation,
                                                                       FRAME_CONVENTION fc,
                                                                       FrBody<OffshoreSystemType> *body, double T0,
                                                                       double psi)
        : FrEquilibriumFrame<OffshoreSystemType>(pos, rotation, fc, body) { this->SetSpringDamping(T0, psi); }

    template<typename OffshoreSystemType>
    FrEqFrameSpringDamping<OffshoreSystemType>::FrEqFrameSpringDamping(const Position &pos,
                                                                       const FrUnitQuaternion &quaternion,
                                                                       FRAME_CONVENTION fc,
                                                                       FrBody<OffshoreSystemType> *body, double T0,
                                                                       double psi)
        : FrEquilibriumFrame<OffshoreSystemType>(pos, quaternion, fc, body) { this->SetSpringDamping(T0, psi); }

    template<typename OffshoreSystemType>
    FrEqFrameSpringDamping<OffshoreSystemType>::FrEqFrameSpringDamping(const FrFrame &otherFrame,
                                                                       FrBody<OffshoreSystemType> *body, double T0,
                                                                       double psi)
        : FrEquilibriumFrame<OffshoreSystemType>(otherFrame, body) { this->SetSpringDamping(T0, psi); }

    template<typename OffshoreSystemType>
    void FrEqFrameSpringDamping<OffshoreSystemType>::SetSpringDamping(const double T0, const double psi) {

      m_w0 = 2. * M_PI / T0;
      m_psi = psi;

      m_damping = 2. * m_psi * m_w0;
      m_stiffness = m_w0 * m_w0;
    }

    template<typename OffshoreSystemType>
    void FrEqFrameSpringDamping<OffshoreSystemType>::Compute(double time) {

      if (std::abs(time - m_prevTime) < FLT_EPSILON) return;

      auto bodyPosition = this->m_body->GetCOGPositionInWorld(NWU);
      auto bodyVelocity = this->m_body->GetCOGLinearVelocityInWorld(NWU);
      auto position = this->GetPosition(NWU);

      Force force;
      force = (bodyPosition - position) * m_stiffness + (bodyVelocity - this->m_velocity) * m_damping;
      force.GetFz() = 0.;

      double temp1, temp2;
      double bodyPsi, psi;
      this->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, psi, NWU);
      this->m_body->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyPsi, NWU);
      auto bodyAngularVelocity = this->m_body->GetAngularVelocityInWorld(NWU).GetWz();

      double torque;
      torque = (bodyPsi - psi) * m_stiffness + (bodyAngularVelocity - this->m_angularVelocity) * m_damping;

      this->m_velocity += force * (time - m_prevTime);
      position += this->m_velocity * (time - m_prevTime);

      this->m_angularVelocity += torque * (time - m_prevTime);

      this->SetPosition(position, NWU);
      SetRotation(this->GetRotation().RotZ_RADIANS(this->m_angularVelocity * (time - m_prevTime), NWU));

      m_prevTime = time;
    }

    // ----------------------------------------------------------------
    // Equilibrium frame with updated mean velocity
    // ----------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrEqFrameMeanMotion<OffshoreSystemType>::FrEqFrameMeanMotion(const Position &pos, const FrRotation &rotation,
                                                                 FRAME_CONVENTION fc,
                                                                 FrBody<OffshoreSystemType> *body,
                                                                 double timePersistence, double timeStep)
        : FrEquilibriumFrame<OffshoreSystemType>(pos, rotation, fc, body) {
      this->SetRecorders(timePersistence, timeStep);
    }

    template<typename OffshoreSystemType>
    FrEqFrameMeanMotion<OffshoreSystemType>::FrEqFrameMeanMotion(const Position &pos,
                                                                 const FrUnitQuaternion &quaternion,
                                                                 FRAME_CONVENTION fc,
                                                                 FrBody<OffshoreSystemType> *body,
                                                                 double timePersistence, double timeStep)
        : FrEquilibriumFrame<OffshoreSystemType>(pos, quaternion, fc, body) {
      this->SetRecorders(timePersistence, timeStep);
    }

    template<typename OffshoreSystemType>
    FrEqFrameMeanMotion<OffshoreSystemType>::FrEqFrameMeanMotion(const FrFrame &otherFrame,
                                                                 FrBody<OffshoreSystemType> *body,
                                                                 double timePersistence,
                                                                 double timeStep)
        : FrEquilibriumFrame<OffshoreSystemType>(otherFrame, body) { this->SetRecorders(timePersistence, timeStep); }

    template<typename OffshoreSystemType>
    FrEqFrameMeanMotion<OffshoreSystemType>::FrEqFrameMeanMotion(FrBody<OffshoreSystemType> *body,
                                                                 double timePersistence, double timeStep, bool initPos)
        : FrEquilibriumFrame<OffshoreSystemType>(body, initPos) { this->SetRecorders(timePersistence, timeStep); }

    template<typename OffshoreSystemType>
    void FrEqFrameMeanMotion<OffshoreSystemType>::SetRecorders(double timePersistence, double timeStep) {
      m_TrSpeedRec = std::make_unique<FrTimeRecorder<Velocity>>(timePersistence, timeStep);
      m_TrSpeedRec->Initialize();
      m_AglSpeedRec = std::make_unique<FrTimeRecorder<double>>(timePersistence, timeStep);
      m_AglSpeedRec->Initialize();
    }

    template<typename OffshoreSystemType>
    void FrEqFrameMeanMotion<OffshoreSystemType>::SetPositionCorrection(double timePersistence, double timeStep,
                                                                        double posCoeff, double angleCoeff) {
      m_ErrPositionRec = std::make_unique<FrTimeRecorder<Position>>(timePersistence, timeStep);
      m_ErrPositionRec->Initialize();
      m_ErrAngleRec = std::make_unique<FrTimeRecorder<double>>(timePersistence, timeStep);
      m_ErrAngleRec->Initialize();
      m_errPosCoeff = posCoeff;
      m_errAngleCoeff = angleCoeff;
    }

    template<typename OffshoreSystemType>
    void FrEqFrameMeanMotion<OffshoreSystemType>::Compute(double time) {

      if (std::abs(time - m_prevTime) < FLT_EPSILON) return;

      m_TrSpeedRec->Record(time, this->m_body->GetCOGLinearVelocityInWorld(NWU));
      m_AglSpeedRec->Record(time, this->m_body->GetAngularVelocityInWorld(NWU).GetWz());

      this->m_velocity = m_TrSpeedRec->GetMean();
      this->m_angularVelocity = m_AglSpeedRec->GetMean();

      auto position = this->GetPosition(NWU);
      position += this->m_velocity * (time - m_prevTime);

      auto angle = this->m_angularVelocity * (time - m_prevTime);

      if (m_ErrPositionRec and m_ErrAngleRec) {

        m_ErrPositionRec->Record(time, this->m_body->GetCOGPositionInWorld(NWU) - this->GetPosition(NWU));
        auto errMeanPosition = m_ErrPositionRec->GetMean();
        position += errMeanPosition * m_errPosCoeff;

        double temp1, temp2, bodyAngle, frameAngle;
        this->m_body->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyAngle, NWU);
        this->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, frameAngle, NWU);
        m_ErrAngleRec->Record(time, bodyAngle - frameAngle);
        auto errMeanAngle = m_ErrAngleRec->GetMean();
        angle += errMeanAngle * m_errAngleCoeff;
      }

      this->SetPosition(position, NWU);
      SetRotation(this->GetRotation().RotZ_RADIANS(angle, NWU));

      m_prevTime = time;
    }

}  // end namespace frydom
