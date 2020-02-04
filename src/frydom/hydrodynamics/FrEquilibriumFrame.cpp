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
#include "frydom/logging/FrLogManager.h"
#include "frydom/logging/FrTypeNames.h"


namespace frydom {

  // ---------------------------------------------------------------------
  // Equilibrium frame
  // ---------------------------------------------------------------------

  FrEquilibriumFrame::FrEquilibriumFrame(const std::string &name, FrBody *body,
                                         const Position &localPos, const double &rot, FRAME_CONVENTION fc)
      : FrLoggable(name, TypeToString(this), body),
        FrPrePhysicsItem(),
        m_velocity(),
        m_angularVelocity(0.),
        m_frame(),
        c_prevTime(0.),
        m_initSpeedFromBody(false) {
    m_bodyNode = GetBody()->NewNode("equilibrium_frame_node_on_body");
    SetEqFramePositionOrientation(localPos, rot, fc);
  }

  FrBody *FrEquilibriumFrame::GetBody() const {
    return GetParent();
  }

  void FrEquilibriumFrame::InitializeVelocityFromBody(bool is_init) {
    m_initSpeedFromBody = is_init;
  }

  void FrEquilibriumFrame::SetEqFramePositionOrientation() {
    // Frame
    m_frame.SetIdentity();
    m_frame.SetPosition(m_bodyNode->GetPositionInWorld(NWU), NWU);
    // Project to the horizontal plane
    ApplyFrameProjection();
    // Node
    m_bodyNode->SetFrameInWorld(m_frame);
  }

  void
  FrEquilibriumFrame::SetEqFramePositionOrientation(const Position &localPos, const double &rot, FRAME_CONVENTION fc) {
    m_bodyNode->SetPositionInBody(localPos, fc);
    // Frame
    m_frame.SetIdentity();
    m_frame.SetPosition(m_bodyNode->GetPositionInWorld(NWU), NWU);
    // Project to the horizontal plane
    ApplyFrameProjection();
    m_frame.RotZ_RADIANS(rot, fc, true);
    // Node
    m_bodyNode->SetFrameInWorld(m_frame);
  }

  Position FrEquilibriumFrame::GetPositionInWorld(FRAME_CONVENTION fc) const {
    return m_frame.GetPosition(fc);
  }

  FrRotation FrEquilibriumFrame::GetRotation() const {
    return m_frame.GetRotation();
  }

  FrFrame FrEquilibriumFrame::GetFrame() const {
    return m_frame;
  }

  void FrEquilibriumFrame::SetVelocityInWorld(const Velocity &velocity, FRAME_CONVENTION fc) {
    if (IsNED(fc)) internal::SwapFrameConvention(velocity);
    m_velocity = velocity;
    m_initSpeedFromBody = false;
  }

  void FrEquilibriumFrame::SetVelocityInFrame(const Velocity &frameVel, FRAME_CONVENTION fc) {
    auto worldVel = m_frame.ProjectVectorFrameInParent(frameVel, fc);
    this->SetVelocityInWorld(worldVel, NWU);
    m_initSpeedFromBody = false;
  }

  void FrEquilibriumFrame::SetAngularVelocity(const double &angularVelocity, FRAME_CONVENTION fc) {
    m_angularVelocity = angularVelocity;
    if (IsNED(fc)) { m_angularVelocity = -m_angularVelocity; }
  }

  Velocity FrEquilibriumFrame::GetFrameVelocityInWorld(FRAME_CONVENTION fc) const {
    Velocity velocity = m_velocity;
    if (IsNED(fc)) internal::SwapFrameConvention(velocity);
    return velocity;
  }

  Velocity FrEquilibriumFrame::GetFrameVelocityInFrame(FRAME_CONVENTION fc) const {
    return m_frame.ProjectVectorParentInFrame<Velocity>(m_velocity, fc);
  }

  FrFrame FrEquilibriumFrame::GetPerturbationFrame() {
    return m_frame.GetInverse() * m_bodyNode->GetFrameInWorld();
  }

  Velocity FrEquilibriumFrame::GetPerturbationVelocityInWorld(FRAME_CONVENTION fc) const {
    auto frameVelocity = this->GetFrameVelocityInWorld(fc);
    auto bodyVelocity = m_bodyNode->GetVelocityInWorld(fc);
    return bodyVelocity - frameVelocity;
  }

  Velocity FrEquilibriumFrame::GetPerturbationVelocityInFrame(FRAME_CONVENTION fc) const {
    auto velocityInWorld = this->GetPerturbationVelocityInWorld(fc);
    return m_frame.ProjectVectorParentInFrame<Velocity>(velocityInWorld, fc);
  }

  GeneralizedVelocity FrEquilibriumFrame::GetPerturbationGeneralizedVelocityInWorld(FRAME_CONVENTION fc) const {
    auto velocity = GetPerturbationVelocityInWorld(fc);
    auto angularVelocity = GetPerturbationAngularVelocity(fc);
    return GeneralizedVelocity(velocity, angularVelocity);
  }

  GeneralizedVelocity FrEquilibriumFrame::GetPerturbationGeneralizedVelocityInFrame(FRAME_CONVENTION fc) const {
    auto velocity = GetPerturbationVelocityInFrame(fc);
    auto angularVelocity = GetPerturbationAngularVelocityInFrame(fc);
    return GeneralizedVelocity(velocity, angularVelocity);
  }

  AngularVelocity FrEquilibriumFrame::GetFrameAngularVelocity(FRAME_CONVENTION fc) const {
    auto w_vel = m_angularVelocity;
    if (IsNED(fc)) { w_vel *= -1.; }
    return {0., 0., w_vel};
  }

  AngularVelocity FrEquilibriumFrame::GetPerturbationAngularVelocity(FRAME_CONVENTION fc) const {
    auto frameAngularVelocity = GetFrameAngularVelocity(fc);
    auto bodyAngularVelocity = m_bodyNode->GetAngularVelocityInWorld(fc);
    return bodyAngularVelocity - frameAngularVelocity;
  }

  AngularVelocity FrEquilibriumFrame::GetPerturbationAngularVelocityInFrame(FRAME_CONVENTION fc) const {
    auto worldAngularVelocity = this->GetPerturbationAngularVelocity(fc);
    return m_frame.ProjectVectorParentInFrame<AngularVelocity>(worldAngularVelocity, fc);
  }

  void FrEquilibriumFrame::Initialize() {
    c_prevTime = 0.;
    if (m_initSpeedFromBody) {
      m_velocity = m_bodyNode->GetVelocityInWorld(NWU);
    }
  }

  void FrEquilibriumFrame::Compute(double time) {

    if (std::abs(time - c_prevTime) < FLT_EPSILON) {
      return;
    }

    auto dt = time - c_prevTime;
    auto prevPosition = m_frame.GetPosition(NWU);

    if (m_velocity.squaredNorm() > FLT_EPSILON) {
      m_frame.SetPosition(prevPosition + m_velocity * dt, NWU);
    }

    if (std::abs(m_angularVelocity) > FLT_EPSILON) {
      m_frame.RotZ_RADIANS(m_angularVelocity * dt, NWU, true);
    }

    c_prevTime = time;
  }

  void FrEquilibriumFrame::StepFinalize() {
    FrPhysicsItem::StepFinalize();
  }

  void FrEquilibriumFrame::DefineLogMessages() {

    auto msg = NewMessage("State", "Equilibrium frame message");

    msg->AddField<double>("time", "s", "Current time of the simulation",
                          [this]() { return GetSystem()->GetTime(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("Position", "m", fmt::format("Equilibrium frame position in the world reference frame in {}", GetLogFC()),
         [this]() { return m_frame.GetPosition(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("CardanAngles", "rad",
         fmt::format("Equilibrium frame orientation in the world reference frame {}", GetLogFC()),
         [this]() {
           double phi, theta, psi;
           m_frame.GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, GetLogFC());
           return Vector3d<double>(phi, theta, psi);
         });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("VelocityInWorld", "m/s",
         fmt::format("Equilibrium frame velocity in the world reference frame in {}", GetLogFC()),
         [this]() { return GetFrameVelocityInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("AngularVelocity", "rad/s",
         fmt::format("Equilibrium frame angular velocity in world reference frame in {}", GetLogFC()),
         [this]() { return GetFrameAngularVelocity(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("PerturbationPosition", "m", fmt::format(
            "Perturbation position between the equilibrium frame and the body in the world reference frame in {}",
            GetLogFC()),
         [this]() { return GetPerturbationFrame().GetPosition(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("PerturbationOrientation", "rad", fmt::format(
            "Perturbation orientation between the equilibrium frame and the body frame in the world reference frame in {}",
            GetLogFC()),
         [this]() {
           double phi, theta, psi;
           GetPerturbationFrame().GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, GetLogFC());
           return Vector3d<double>(phi, theta, psi);
         });
  }

  void FrEquilibriumFrame::ApplyFrameProjection() {
    m_frame.GetRotation().SetNullRotation();
    // Apply same yaw as body rotation
    double phi, theta, psi;
    GetBody()->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
    m_frame.RotZ_RADIANS(psi, NWU, true);
  }

  void FrEquilibriumFrame::SetAngleRotation(const double &angle, FRAME_CONVENTION fc) {
    m_frame.RotZ_RADIANS(angle, fc, true);
  }

  // Makers

  std::shared_ptr<FrEquilibriumFrame>
  make_equilibrium_frame(const std::string &name, FrOffshoreSystem *system, const std::shared_ptr<FrBody> &body,
                         const Position &localPos, const double &rot, FRAME_CONVENTION fc) {
    auto eqframe = std::make_shared<FrEquilibriumFrame>(name, body.get(), localPos, rot, fc);
    system->Add(eqframe);
    return eqframe;
  }

  std::shared_ptr<FrEquilibriumFrame>
  make_equilibrium_frame(const std::string &name, FrOffshoreSystem *system, const std::shared_ptr<FrBody> &body) {
    auto eqFrame = std::make_shared<FrEquilibriumFrame>(name, body.get(), body->GetCOG(NWU), 0., NWU);
    system->Add(eqFrame);
    return eqFrame;
  }

  // -----------------------------------------------------------------------
  // Equilibrium frame with spring damping restoring force
  // -----------------------------------------------------------------------

  FrEqFrameSpringDamping::FrEqFrameSpringDamping(const std::string &name,
                                                 FrBody *body,
                                                 const Position &localPos,
                                                 const double &rot,
                                                 FRAME_CONVENTION fc,
                                                 double cutoffTime,
                                                 double dampingRatio)
      : FrEquilibriumFrame(name, body, localPos, rot, fc) {
    this->SetSpringDamping(cutoffTime, dampingRatio);
  }

  void FrEqFrameSpringDamping::Compute(double time) {

    if (std::abs(time - m_prevTime) < FLT_EPSILON) return;

    auto bodyPosition = m_bodyNode->GetPositionInWorld(NWU);
    auto bodyVelocity = m_bodyNode->GetVelocityInWorld(NWU);
    auto position = m_frame.GetPosition(NWU);

    Force force;
    force = (bodyPosition - position) * m_stiffness + (bodyVelocity - m_velocity) * m_damping;
    force.GetFz() = 0.;

    double temp1, temp2;
    double bodyPsi, psi;
    m_frame.GetRotation().GetCardanAngles_RADIANS(temp1, temp2, psi, NWU);
    m_bodyNode->GetFrameInWorld().GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyPsi, NWU);
    auto bodyAngularVelocity = m_bodyNode->GetAngularVelocityInWorld(NWU).GetWz();

    double torque;
    torque = (bodyPsi - psi) * m_stiffness + (bodyAngularVelocity - m_angularVelocity) * m_damping;

    m_velocity += force * (time - m_prevTime);
    position += m_velocity * (time - m_prevTime);

    m_angularVelocity += torque * (time - m_prevTime);

    m_frame.SetPosition(position, NWU);
    m_frame.SetRotation(m_frame.GetRotation().RotZ_RADIANS(m_angularVelocity * (time - m_prevTime), NWU));

    m_prevTime = time;
  }

  void FrEqFrameSpringDamping::SetSpringDamping(double cutoffTime, double dampingRatio) {

    auto w0 = 2. * MU_PI / cutoffTime;
    m_damping = 2. * dampingRatio * w0;
    m_stiffness = w0 * w0;

  }

  // Makers

  std::shared_ptr<FrEqFrameSpringDamping>
  make_spring_damping_equilibrium_frame(const std::string &name,
                                        const std::shared_ptr<FrBody> &body,
                                        const Position &localPos,
                                        const double &rot,
                                        FRAME_CONVENTION fc,
                                        FrOffshoreSystem *system,
                                        double cutoffTime,
                                        double dampingRatio) {
    auto eqframe = std::make_shared<FrEqFrameSpringDamping>(name, body.get(),
                                                            localPos, rot, fc, cutoffTime, dampingRatio);
    system->Add(eqframe);
    return eqframe;
  }

  std::shared_ptr<FrEqFrameSpringDamping>
  make_spring_damping_equilibrium_frame(const std::string &name,
                                        const std::shared_ptr<FrBody> &body,
                                        FrOffshoreSystem *system,
                                        double cutoffTime,
                                        double dampingRatio) {
    auto eqFrame = std::make_shared<FrEqFrameSpringDamping>(name, body.get(),
                                                            body->GetCOG(NWU), 0., NWU, cutoffTime, dampingRatio);
    system->Add(eqFrame);
    return eqFrame;
  }

  // ----------------------------------------------------------------
  // Equilibrium frame with updated mean velocity
  // ----------------------------------------------------------------

  FrEqFrameMeanMotion::FrEqFrameMeanMotion(const std::string &name,
                                           FrBody *body,
                                           const Position &localPos,
                                           const double &rot,
                                           FRAME_CONVENTION fc,
                                           double timePersistence,
                                           double timeStep)
      : FrEquilibriumFrame(name, body, localPos, rot, fc) {
    this->SetRecorders(timePersistence, timeStep);
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

    m_TrSpeedRec->Record(time, m_bodyNode->GetVelocityInWorld(NWU));
    m_AglSpeedRec->Record(time, m_bodyNode->GetAngularVelocityInWorld(NWU).GetWz());

    m_velocity = m_TrSpeedRec->GetMean();
    m_angularVelocity = m_AglSpeedRec->GetMean();

    auto position = m_frame.GetPosition(NWU);
    position += m_velocity * (time - m_prevTime);

    auto angle = m_angularVelocity * (time - m_prevTime);

    if (m_ErrPositionRec and m_ErrAngleRec) {

      m_ErrPositionRec->Record(time, m_bodyNode->GetPositionInWorld(NWU) - m_frame.GetPosition(NWU));
      auto errMeanPosition = m_ErrPositionRec->GetMean();
      position += errMeanPosition * m_errPosCoeff;

      double temp1, temp2, bodyAngle, frameAngle;
      m_bodyNode->GetFrameInWorld().GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyAngle, NWU);
      m_frame.GetRotation().GetCardanAngles_RADIANS(temp1, temp2, frameAngle, NWU);
      m_ErrAngleRec->Record(time, bodyAngle - frameAngle);
      auto errMeanAngle = m_ErrAngleRec->GetMean();
      angle += errMeanAngle * m_errAngleCoeff;
    }

    m_frame.SetPosition(position, NWU);
    m_frame.SetRotation(m_frame.GetRotation().RotZ_RADIANS(angle, NWU));

    m_prevTime = time;
  }

  void FrEqFrameMeanMotion::SetRecorders(double timePersistence, double timeStep) {
    m_TrSpeedRec = std::make_unique<FrTimeRecorder<Velocity>>(timePersistence, timeStep);
    m_TrSpeedRec->Initialize();
    m_AglSpeedRec = std::make_unique<FrTimeRecorder<double>>(timePersistence, timeStep);
    m_AglSpeedRec->Initialize();
  }

  // Makers

  std::shared_ptr<FrEqFrameMeanMotion>
  make_mean_motion_equilibrium_frame(const std::string &name,
                                     FrOffshoreSystem *system,
                                     const std::shared_ptr<FrBody> &body,
                                     double timePersistence,
                                     double timeStep) {
    auto eqframe = std::make_shared<FrEqFrameMeanMotion>(name, body.get(),
                                                         body->GetCOG(NWU), 0., NWU, timePersistence, timeStep);
    system->Add(eqframe);
    return eqframe;
  }

  std::shared_ptr<FrEqFrameMeanMotion>
  make_mean_motion_equilibrium_frame(const std::string &name,
                                     FrOffshoreSystem *system,
                                     const std::shared_ptr<FrBody> &body,
                                     const Position &localPos,
                                     const double &rot,
                                     FRAME_CONVENTION fc,
                                     double timePersistence,
                                     double timeStep) {
    auto eqframe = std::make_shared<FrEqFrameMeanMotion>(name, body.get(),
                                                         localPos, rot, fc, timePersistence, timeStep);
    system->Add(eqframe);
    return eqframe;
  }

}  // end namespace frydom
