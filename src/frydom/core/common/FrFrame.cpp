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


#include "FrFrame.h"
#include "frydom/core/math/FrMatrix.h"

#include "chrono/core/ChMatrixDynamic.h" // TODO : voir pourquoi on doit avoir cet include...


namespace frydom {


  FrFrame::FrFrame() : m_chronoFrame() {};  // OK

  FrFrame::FrFrame(const Position &pos, const FrRotation &rotation, FRAME_CONVENTION fc) {  // OK
    SetPosition(pos, fc);
    SetRotation(rotation);
  }

  FrFrame::FrFrame(const Position &pos, const frydom::FrUnitQuaternion &quaternion, FRAME_CONVENTION fc) {  // OK
    SetPosition(pos, fc);
    SetRotation(quaternion);
  }

  FrFrame::FrFrame(const FrFrame &otherFrame) {  // OK
    m_chronoFrame = otherFrame.m_chronoFrame;
  }

  void FrFrame::SetPosition(double x, double y, double z, FRAME_CONVENTION fc) {  // OK

    if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z); // Express In NWU
    m_chronoFrame.SetPos(chrono::ChVector<double>(x, y, z));
  }

  void FrFrame::SetPosition(const Position &position, FRAME_CONVENTION fc) {  // OK
    auto posTmp = position;
    if (IsNED(fc)) internal::SwapFrameConvention<Position>(posTmp);
    m_chronoFrame.SetPos(internal::Vector3dToChVector(posTmp));
  }

  void FrFrame::GetPosition(double &x, double &y, double &z, FRAME_CONVENTION fc) const {  // OK
    auto pos = m_chronoFrame.GetPos();  // In NWU
    x = pos.x();
    y = pos.y();
    z = pos.z();

    if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);
  }

  void FrFrame::GetPosition(Position &position, FRAME_CONVENTION fc) const {  // OK
    position = GetPosition(fc);
  }

  Position FrFrame::GetPosition(FRAME_CONVENTION fc) const {  // OK
    auto pos = internal::ChVectorToVector3d<Position>(m_chronoFrame.GetPos()); // In NWU
    if (IsNED(fc)) internal::SwapFrameConvention<Position>(pos);
    return pos;
  }

  void FrFrame::SetX(double x, FRAME_CONVENTION fc) {  // OK
    m_chronoFrame.GetPos()[0] = x;
  }

  void FrFrame::SetY(double y, FRAME_CONVENTION fc) {  // OK
    if (IsNED(fc)) y = -y;
    m_chronoFrame.GetPos()[1] = y;
  }

  void FrFrame::SetZ(double z, FRAME_CONVENTION fc) {  // OK
    if (IsNED(fc)) z = -z;
    m_chronoFrame.GetPos()[2] = z;
  }

  double FrFrame::GetX(FRAME_CONVENTION fc) const {  // OK
    return m_chronoFrame.GetPos()[0];
  }

  double FrFrame::GetY(FRAME_CONVENTION fc) const {  // OK
    double y = m_chronoFrame.GetPos()[1];
    if (IsNED(fc)) y = -y;
    return y;
  }

  double FrFrame::GetZ(FRAME_CONVENTION fc) const {  // OK
    double z = m_chronoFrame.GetPos()[2];
    if (IsNED(fc)) z = -z;
    return z;
  }

  void FrFrame::Set(Position pos, Direction e1, Direction e2, Direction e3, FRAME_CONVENTION fc) {
    SetPosition(pos, fc);

    mathutils::Matrix33<double> matrix;
    matrix << e1.Getux(), e2.Getux(), e3.Getux(),
        e1.Getuy(), e2.Getuy(), e3.Getuy(),
        e1.Getuz(), e2.Getuz(), e3.Getuz();

    m_chronoFrame.SetRot(internal::Matrix33ToChMatrix33(matrix));
  }

  void FrFrame::SetRotation(const FrRotation &rotation) {  // OK
    SetRotation(rotation.GetQuaternion());
  }

  void FrFrame::SetRotation(const FrUnitQuaternion &quaternion) {  // OK
    m_chronoFrame.SetRot(internal::Fr2ChQuaternion(quaternion));
  }

  void FrFrame::SetNoRotation() {  // OK
    auto curPos = m_chronoFrame.GetPos();
    m_chronoFrame.SetIdentity();
    m_chronoFrame.SetPos(curPos);
  }

  void FrFrame::SetNoTranslation() {  // OK
    m_chronoFrame.GetPos().SetNull();
  }

  void FrFrame::SetIdentity() {  // OK
    m_chronoFrame.SetIdentity();
  }

  FrRotation FrFrame::GetRotation() const {  // OK
    return FrRotation(GetQuaternion());
  }

  FrUnitQuaternion FrFrame::GetQuaternion() const {  // OK
    return internal::Ch2FrQuaternion(m_chronoFrame.GetRot());  // In NWU
  }

  FrFrame FrFrame::operator*(const FrFrame &otherFrame) const {
    auto newFrame = FrFrame();
    newFrame.m_chronoFrame = m_chronoFrame * otherFrame.m_chronoFrame;
    return newFrame;
  }

  void FrFrame::operator*=(const FrFrame &otherFrame) {
    m_chronoFrame >>= otherFrame.m_chronoFrame;  // TODO : verifier !!
  }

  bool FrFrame::operator==(const FrFrame &otherFrame) const {
    return GetPosition(NWU) == otherFrame.GetPosition(NWU) && GetQuaternion() == otherFrame.GetQuaternion();
  }

  bool FrFrame::operator!=(const FrFrame &otherFrame) const {
    return !(*this == otherFrame);
  }

  bool FrFrame::IsApprox(const FrFrame &otherFrame, const double &prec) const {
    return GetPosition(NWU).isApprox(otherFrame.GetPosition(NWU), prec) &&
           GetQuaternion().IsApprox(otherFrame.GetQuaternion(), prec);
  }

  bool FrFrame::IsZero(const double &prec) const {
    return GetPosition(NWU).isZero(prec) && GetRotation().GetQuaternion().IsZero(prec);
  }

  FrFrame FrFrame::GetOtherFrameRelativeTransform_WRT_ThisFrame(const frydom::FrFrame &otherFrame) const {  // OK
    return this->GetInverse() * otherFrame;
  }

  FrFrame FrFrame::GetThisFrameRelativeTransform_WRT_OtherFrame(const frydom::FrFrame &otherFrame) const {  // OK
    return GetOtherFrameRelativeTransform_WRT_ThisFrame(otherFrame).GetInverse();
  }

  std::ostream &FrFrame::cout(std::ostream &os) const {  // OK

    double x, y, z;
    GetPosition(x, y, z, NWU);

    os << std::endl;
    os << "Frame :\n";
    os << "-------\n";
    os << "Translation (m, In NWU): ";
    os << "X = " << x;
    os << "; Y = " << y;
    os << "; Z = " << z;
    os << std::endl;
    os << GetRotation();
    os << std::endl;

    return os;

  }

  std::ostream &operator<<(std::ostream &os, const FrFrame &frame) {  // OK
    return frame.cout(os);
  }

  void FrFrame::RotateInFrame(const FrUnitQuaternion &quaternion) {
    m_chronoFrame.SetRot(m_chronoFrame.GetRot() * internal::Fr2ChQuaternion(quaternion));
  }

  void FrFrame::RotateInFrame(const FrRotation &rotation) {
    RotateInFrame(rotation.GetQuaternion());
  }

  void FrFrame::RotateInFrame(const Direction &direction, double angleRad, FRAME_CONVENTION fc) {
    RotateInFrame(FrUnitQuaternion(direction, angleRad, fc));
  }

  void FrFrame::RotateInFrame(double phiRad, double thetaRad, double psiRad, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {
    FrRotation rotation;
    rotation.SetEulerAngles_DEGREES(phiRad, thetaRad, psiRad, seq, fc);
    RotateInFrame(rotation);
  }

  void FrFrame::RotateInParent(const FrUnitQuaternion &quaternion) {
    m_chronoFrame.SetRot(internal::Fr2ChQuaternion(quaternion) * m_chronoFrame.GetRot());
  }

  void FrFrame::RotateInParent(const FrRotation &rotation) {
    RotateInParent(rotation.GetQuaternion());
  }

  void FrFrame::RotateInParent(const Direction &direction, double angleRad, FRAME_CONVENTION fc) {
    RotateInParent(FrUnitQuaternion(direction, angleRad, fc));
  }

  void FrFrame::RotateInParent(double phiRad, double thetaRad, double psiRad, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {
    FrRotation rotation;
    rotation.SetEulerAngles_DEGREES(phiRad, thetaRad, psiRad, seq, fc);
    RotateInParent(rotation);
  }

  void FrFrame::TranslateInFrame(const Translation &translation, FRAME_CONVENTION fc) {
    TranslateInParent(ProjectVectorFrameInParent<Translation>(translation, fc), fc);
  }

  void FrFrame::TranslateInFrame(const Direction &direction, double distance, FRAME_CONVENTION fc) {
    assert(direction.IsUnit());
    TranslateInFrame(direction * distance, fc);
  }

  void FrFrame::TranslateInFrame(double x, double y, double z, FRAME_CONVENTION fc) {
    TranslateInFrame(Translation(x, y, z), fc);
  }

  void FrFrame::TranslateInParent(const Translation &translation, FRAME_CONVENTION fc) {
    auto tmpTranslation = translation;
    if (IsNED(fc)) internal::SwapFrameConvention<Translation>(tmpTranslation);
    m_chronoFrame.Move(internal::Vector3dToChVector(tmpTranslation));
  }

  void FrFrame::TranslateInParent(const Direction &direction, double distance, FRAME_CONVENTION fc) {
    assert(direction.IsUnit());
    TranslateInParent(direction * distance, fc);
  }

  void FrFrame::TranslateInParent(double x, double y, double z, FRAME_CONVENTION fc) {
    TranslateInParent(Translation(x, y, z), fc);
  }


  // FIXME : c'est une multiplication a gauche !
  // FIXME : changer les implementations de tous les RotX etc... et reposer sur des methodes de FrRotation et FrQuaternion !!!!
  void FrFrame::RotX_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis) {
    chrono::ChVector<double> axis;
    if (localAxis) {
      axis = m_chronoFrame.GetRot().GetXaxis();  // In NWU
    } else {
      axis = chrono::ChVector<double>(1., 0., 0.);
    }

    m_chronoFrame.SetRot(chrono::Q_from_AngAxis(angle, axis) * m_chronoFrame.GetRot());
  }

  void FrFrame::RotX_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis) {
    RotX_RADIANS(angle * DEG2RAD, fc, localAxis);
  }

  void FrFrame::RotY_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis) {
    chrono::ChVector<double> axis;
    if (localAxis) {
      axis = m_chronoFrame.GetRot().GetYaxis();  // In NWU
    } else {
      axis = chrono::ChVector<double>(0., 1., 0.);
    }

    if (IsNED(fc)) angle = -angle;

    m_chronoFrame.SetRot(chrono::Q_from_AngAxis(angle, axis) * m_chronoFrame.GetRot());
  }

  void FrFrame::RotY_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis) {
    RotY_RADIANS(angle * DEG2RAD, fc, localAxis);
  }

  void FrFrame::RotZ_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis) {
    chrono::ChVector<double> axis;
    if (localAxis) {
      axis = m_chronoFrame.GetRot().GetZaxis();  // In NWU
    } else {
      axis = chrono::ChVector<double>(0., 0., 1.);
    }

    if (IsNED(fc)) angle = -angle;

    m_chronoFrame.SetRot(chrono::Q_from_AngAxis(angle, axis) * m_chronoFrame.GetRot());
  }

  void FrFrame::RotZ_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis) {
    RotZ_RADIANS(angle * DEG2RAD, fc, localAxis);
  }

  void FrFrame::SetRotX_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
    SetIdentity();
    RotX_RADIANS(angle, fc, false);
  }

  void FrFrame::SetRotX_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
    SetIdentity();
    RotX_DEGREES(angle, fc, false);
  }

  void FrFrame::SetRotY_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
    SetIdentity();
    RotY_RADIANS(angle, fc, false);
  }

  void FrFrame::SetRotY_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
    SetIdentity();
    RotY_DEGREES(angle, fc, false);
  }

  void FrFrame::SetRotZ_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
    SetIdentity();
    RotZ_RADIANS(angle, fc, false);
  }

  void FrFrame::SetRotZ_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
    SetIdentity();
    RotZ_DEGREES(angle, fc, false);
  }

  FrFrame &FrFrame::Inverse() {  // OK
    m_chronoFrame.Invert();
    return *this;
  }

  FrFrame FrFrame::GetInverse() const {  // OK
    return internal::ChFrame2FrFrame(m_chronoFrame.GetInverse());
  }

  FrFrame FrFrame::ProjectToXYPlane(FRAME_CONVENTION fc) const {

    Direction xaxis = GetXAxisInParent(fc);
    xaxis.z() = 0.;
    xaxis.normalize();

    Direction yaxis = GetYAxisInParent(fc);
    yaxis.z() = 0.;
    yaxis.normalize();

    Direction zaxis = Direction(0., 0., 1.);
    Position origin = GetPosition(fc);

    return FrFrame(origin, FrRotation(xaxis, yaxis, zaxis, fc), fc);
  }

  Direction FrFrame::GetXAxisInParent(FRAME_CONVENTION fc) const {
    return GetQuaternion().GetXAxis(fc);
  }

  Direction FrFrame::GetYAxisInParent(FRAME_CONVENTION fc) const {
    return GetQuaternion().GetYAxis(fc);
  }

  Direction FrFrame::GetZAxisInParent(FRAME_CONVENTION fc) const {
    return GetQuaternion().GetZAxis(fc);
  }

  Position FrFrame::GetPointPositionInParent(const Position &framePos, FRAME_CONVENTION fc) const {

    return GetPosition(fc) + ProjectVectorFrameInParent<Position>(framePos, fc);

  }

  Position FrFrame::GetPointPositionInFrame(const Position &framePos, FRAME_CONVENTION fc) const {
    Position pos = framePos - GetPosition(fc);
    return ProjectVectorParentInFrame(pos, fc);
  }


}  // end namespace frydom
