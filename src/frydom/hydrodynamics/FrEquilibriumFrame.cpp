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

  FrEquilibriumFrame::FrEquilibriumFrame(const std::string &name, FrBody *body)
      : FrLoggable(name, TypeToString(this), body),
        FrPrePhysicsItem(),
        m_velocity(),
        m_angularVelocity(0.),
        m_frame(),
        c_prevTime(0.),
        m_initPositionFromBody(true),
        m_initSpeedFromBody(false) {}


  FrBody *FrEquilibriumFrame::GetBody() const {
    return GetParent();
  }

  void FrEquilibriumFrame::SetPositionToBodyCOGPosition() {
    m_frame.SetPosition(GetBody()->GetCOGPositionInWorld(NWU), NWU);
    m_frame.SetRotation(GetBody()->GetRotation());
    m_initPositionFromBody = false;
  }

  void FrEquilibriumFrame::SetPositionInWorld(Position pos, FRAME_CONVENTION fc) {
    m_frame.SetPosition(pos, fc);
    InitPositionFromBody(false);
  }

  Position FrEquilibriumFrame::GetPositionInWorld(FRAME_CONVENTION fc) const {
    return m_frame.GetPosition(fc);
  }

  void FrEquilibriumFrame::SetRotation(const FrRotation &rotation) {
    m_frame.SetRotation(rotation);
    InitPositionFromBody(false);
  }

  FrRotation FrEquilibriumFrame::GetRotation() const {
    return m_frame.GetRotation();
  }

  void FrEquilibriumFrame::SetFrameInWorld(const FrFrame &frame) {
    m_frame = frame;
    InitPositionFromBody(false);
  }

  FrFrame FrEquilibriumFrame::GetFrameInWorld() const {
    return m_frame;
  }

  void FrEquilibriumFrame::SetVelocityToBodyCOGVelocity() {
    m_velocity = GetBody()->GetCOGLinearVelocityInWorld(NWU);
    m_angularVelocity = 0.;
    m_initSpeedFromBody = false;
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

  void FrEquilibriumFrame::SetAngularVelocityAroundZ(const double &angularVelocity, FRAME_CONVENTION fc) {
    m_angularVelocity = angularVelocity;
    if (IsNED(fc)) { m_angularVelocity = -m_angularVelocity; }
    m_initSpeedFromBody = false;
  }

  Velocity FrEquilibriumFrame::GetVelocityInWorld(FRAME_CONVENTION fc) const {
    Velocity velocity = m_velocity;
    if (IsNED(fc)) internal::SwapFrameConvention(velocity);
    return velocity;
  }

  Velocity FrEquilibriumFrame::GetVelocityInFrame(FRAME_CONVENTION fc) const {
    return m_frame.ProjectVectorParentInFrame<Velocity>(m_velocity, fc);
  }

  FrFrame FrEquilibriumFrame::GetPerturbationFrame() {
    return m_frame.GetInverse() * GetBody()->GetFrameAtCOG(NWU);;
  }

  Velocity FrEquilibriumFrame::GetPerturbationVelocityInWorld(FRAME_CONVENTION fc) const {
    auto frameVelocity = this->GetVelocityInWorld(fc);
    auto bodyVelocity = GetBody()->GetCOGLinearVelocityInWorld(fc);
    return bodyVelocity - frameVelocity;
  }

  Velocity FrEquilibriumFrame::GetPerturbationVelocityInFrame(FRAME_CONVENTION fc) const {
    auto velocityInWorld = this->GetPerturbationVelocityInWorld(fc);
    return m_frame.ProjectVectorParentInFrame<Velocity>(velocityInWorld, fc);
  }

  GeneralizedVelocity FrEquilibriumFrame::GetPerturbationGeneralizedVelocityInWorld(FRAME_CONVENTION fc) const {
    auto velocity = GetPerturbationVelocityInWorld(fc);
    auto angularVelocity = GetAngularPerturbationVelocity(fc);
    return GeneralizedVelocity(velocity, angularVelocity);
  }

  GeneralizedVelocity FrEquilibriumFrame::GetPerturbationGeneralizedVelocityInFrame(FRAME_CONVENTION fc) const {
    auto velocity = GetPerturbationVelocityInFrame(fc);
    auto angularVelocity = GetAngularPerturbationVelocityInFrame(fc);
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
    auto bodyAngularVelocity = GetBody()->GetAngularVelocityInWorld(fc);
    return bodyAngularVelocity - frameAngularVelocity;
  }

  AngularVelocity FrEquilibriumFrame::GetAngularPerturbationVelocityInFrame(FRAME_CONVENTION fc) const {
    auto worldAngularVelocity = this->GetAngularPerturbationVelocity(fc);
    return m_frame.ProjectVectorParentInFrame<AngularVelocity>(worldAngularVelocity, fc);
  }

  void FrEquilibriumFrame::Initialize() {

    if (!GetBody()) { throw FrException("error : the body is not defined in equilibrium frame"); }

    if (m_initPositionFromBody) this->SetPositionToBodyCOGPosition();
    if (m_initSpeedFromBody) this->SetVelocityToBodyCOGVelocity();

    c_prevTime = 0.;

    // Declare this object to the log manager
//    GetSystem()->GetLogManager()->Add(this);
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

//    void FrEquilibriumFrame::AddFields() {
//
////        if (IsLogged()) {
////
////            // Add the fields to be logged here
////            m_message->AddField<double>("time", "s", "Current time of the simulation",
////                                        [this]() { return m_system->GetTime(); });
////
////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
////                    ("Position","m", fmt::format("Equilibrium frame position in the world reference frame in {}", GetLogFrameConvention()),
////                     [this]() {return m_frame.GetPosition(GetLogFrameConvention());});
////
////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
////                    ("CardanAngles","rad", fmt::format("Equilibrium frame orientation in the world reference frame in {}", GetLogFrameConvention()),
////                     [this]() {double phi, theta, psi; m_frame.GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, GetLogFrameConvention()); return Vector3d<double>(phi, theta, psi);});
////
////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
////                    ("VelocityInWorld","m/s", fmt::format("Equilibrium frame velocity in the world reference frame in {}", GetLogFrameConvention()),
////                     [this]() {return GetVelocityInWorld(GetLogFrameConvention());});
////
////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
////                    ("AngularVelocity","rad/s", fmt::format("Equilibrium frame angular velocity in the world reference frame in {}", GetLogFrameConvention()),
////                     [this]() {return GetAngularVelocity(GetLogFrameConvention());});
////
////
////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
////                    ("PerturbationPosition","m", fmt::format("Perturbation position between the equilibrium frame and the body frame in the world reference frame in {}", GetLogFrameConvention()),
////                     [this]() {return GetPerturbationFrame().GetPosition(GetLogFrameConvention());});
////
////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
////                    ("PerturbationOrientation","m", fmt::format("Perturbation orientation between the equilibrium frame and the body frame in the world reference frame in {}", GetLogFrameConvention()),
////                     [this]() {double phi, theta, psi;
////                     GetPerturbationFrame().GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, GetLogFrameConvention());
////                     return Vector3d<double>(phi, theta, psi);});
////
////        }
//    }

  void FrEquilibriumFrame::StepFinalize() {

    FrPhysicsItem::StepFinalize();

    // Serialize and send the message log
//        FrObject::SendLog();

  }

  void FrEquilibriumFrame::DefineLogMessages() {
    // TODO
  }

//  FrOffshoreSystem *FrEquilibriumFrame::GetSystem() const {
//    return GetParent()->GetSystem();
//  }


  void FrEquilibriumFrame::InitSpeedFromBody(bool is_init) {
    m_initSpeedFromBody = is_init;
  }

  void FrEquilibriumFrame::InitPositionFromBody(bool is_init) {
    m_initPositionFromBody = is_init;
  }

  std::shared_ptr<FrEquilibriumFrame>
  make_equilibrium_frame(const std::string &name, const std::shared_ptr<FrBody> &body, FrOffshoreSystem *system) {
    auto eqframe = std::make_shared<FrEquilibriumFrame>(name, body.get());
    system->Add(eqframe);
    return eqframe;
  }

  // -----------------------------------------------------------------------
  // Equilibrium frame with spring damping restoring force
  // -----------------------------------------------------------------------

  FrEqFrameSpringDamping::FrEqFrameSpringDamping(const std::string &name,
                                                 FrBody *body,
                                                 double cutoffTime,
                                                 double dampingRatio)
      : FrEquilibriumFrame(name, body) {
    this->SetSpringDamping(cutoffTime, dampingRatio);
  }

  void FrEqFrameSpringDamping::SetSpringDamping(double cutoffTime, double dampingRatio) {

    auto w0 = 2. * MU_PI / cutoffTime;
    m_damping = 2. * dampingRatio * w0;
    m_stiffness = w0 * w0;

  }

  void FrEqFrameSpringDamping::Compute(double time) {

    if (std::abs(time - m_prevTime) < FLT_EPSILON) return;

    auto bodyPosition = GetBody()->GetCOGPositionInWorld(NWU);
    auto bodyVelocity = GetBody()->GetCOGLinearVelocityInWorld(NWU);
    auto position = m_frame.GetPosition(NWU);

    Force force;
    force = (bodyPosition - position) * m_stiffness + (bodyVelocity - m_velocity) * m_damping;
    force.GetFz() = 0.;

    double temp1, temp2;
    double bodyPsi, psi;
    m_frame.GetRotation().GetCardanAngles_RADIANS(temp1, temp2, psi, NWU);
    GetBody()->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyPsi, NWU);
    auto bodyAngularVelocity = GetBody()->GetAngularVelocityInWorld(NWU).GetWz();

    double torque;
    torque = (bodyPsi - psi) * m_stiffness + (bodyAngularVelocity - m_angularVelocity) * m_damping;

    m_velocity += force * (time - m_prevTime);
    position += m_velocity * (time - m_prevTime);

    m_angularVelocity += torque * (time - m_prevTime);

    m_frame.SetPosition(position, NWU);
    m_frame.SetRotation(m_frame.GetRotation().RotZ_RADIANS(m_angularVelocity * (time - m_prevTime), NWU));

    m_prevTime = time;
  }

  std::shared_ptr<FrEqFrameSpringDamping>
  make_spring_damping_equilibrium_frame(const std::string &name,
                                        const std::shared_ptr<FrBody> &body,
                                        FrOffshoreSystem *system,
                                        double cutoffTime,
                                        double dampingRatio) {
    auto eqframe = std::make_shared<FrEqFrameSpringDamping>(name, body.get(), cutoffTime, dampingRatio);
    system->Add(eqframe);
    return eqframe;
  }

  // ----------------------------------------------------------------
  // Equilibrium frame with updated mean velocity
  // ----------------------------------------------------------------

  FrEqFrameMeanMotion::FrEqFrameMeanMotion(const std::string &name,
                                           FrBody *body,
                                           double timePersistence,
                                           double timeStep)
      : FrEquilibriumFrame(name, body) {
    this->SetRecorders(timePersistence, timeStep);
  }


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

    m_TrSpeedRec->Record(time, GetBody()->GetCOGLinearVelocityInWorld(NWU));
    m_AglSpeedRec->Record(time, GetBody()->GetAngularVelocityInWorld(NWU).GetWz());

    m_velocity = m_TrSpeedRec->GetMean();
    m_angularVelocity = m_AglSpeedRec->GetMean();

    auto position = m_frame.GetPosition(NWU);
    position += m_velocity * (time - m_prevTime);

    auto angle = m_angularVelocity * (time - m_prevTime);

    if (m_ErrPositionRec and m_ErrAngleRec) {

      m_ErrPositionRec->Record(time, GetBody()->GetCOGPositionInWorld(NWU) - m_frame.GetPosition(NWU));
      auto errMeanPosition = m_ErrPositionRec->GetMean();
      position += errMeanPosition * m_errPosCoeff;

      double temp1, temp2, bodyAngle, frameAngle;
      GetBody()->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyAngle, NWU);
      m_frame.GetRotation().GetCardanAngles_RADIANS(temp1, temp2, frameAngle, NWU);
      m_ErrAngleRec->Record(time, bodyAngle - frameAngle);
      auto errMeanAngle = m_ErrAngleRec->GetMean();
      angle += errMeanAngle * m_errAngleCoeff;
    }

    m_frame.SetPosition(position, NWU);
    m_frame.SetRotation(m_frame.GetRotation().RotZ_RADIANS(angle, NWU));

    m_prevTime = time;
  }

  std::shared_ptr<FrEqFrameMeanMotion>
  make_mean_motion_equilibrium_frame(const std::string &name,
                                     const std::shared_ptr<FrBody> &body,
                                     FrOffshoreSystem *system,
                                     double timePersistence,
                                     double timeStep) {
    auto eqframe = std::make_shared<FrEqFrameMeanMotion>(name, body.get(), timePersistence, timeStep);
    system->Add(eqframe);
    return eqframe;
  }

}  // end namespace frydom
