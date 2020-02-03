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

#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;

//
// CLASS OF THE TEST
//


class TestFrForce : public FrForce {

 private:
  Position m_PointCOGInWorld;                 ///< Position of the COG in world
  Position m_PointCOGInBody;                  ///< Position of the COG in body
  Position m_PointREFInWorld;                 ///< Position of the body in world

  FrFrame m_frameREF;                        ///< Body frame
  FrUnitQuaternion m_quatREF;                    ///< Rotation of the body in world

  Position m_PointInBody;                     ///< Position of a point of the body in body
  Position m_PointInWorld;                    ///< Position of a point of the boyd in world

  Force m_forceInWorldAtCOG;                  ///< Force at the COG in world frame
  Force m_forceInBodyAtCOG;                   ///< Force at the COG in body frame
  Force m_forceInWorldAtPoint;                ///< Force at a point of the body in world frame
  Force m_forceInBodyAtPoint;                 ///< Force at a point of the body in body frame

  Torque m_torqueInWorldAtCOG;                ///< Torque at COG in world frame
  Torque m_torqueInBodyAtCOG;                 ///< Torque at COG in body frame
  Torque m_torqueInWorldAtPoint;              ///< Torque at a point of the body in world frame
  Torque m_torqueInBodyAtPoint;               ///< Torque at a point of the body in body frame

  Position m_PointInBody_NED;                 ///< Position of a point of the body in body (NED)
  Position m_PointInWorld_NED;                ///< Position of a point of the body in world (NED)

  Force m_forceInWorldAtCOG_NED;              ///< Force at the COG in world frame (NED)
  Force m_forceInBodyAtCOG_NED;               ///< Force at the COG in body frame (NED)
  Force m_forceInWorldAtPoint_NED;            ///< Force at a point of the body in world frame (NED)
  Force m_forceInBodyAtPoint_NED;             ///< Force at a point of the body in body frame (NED)

  Torque m_torqueInWorldAtCOG_NED;            ///< Torque at COG in world frame (NED)
  Torque m_torqueInBodyAtCOG_NED;             ///< Torque at COG in body frame (NED)
  Torque m_torqueInWorldAtPoint_NED;          ///< Torque at a point of the body in world frame (NED)
  Torque m_torqueInBodyAtPoint_NED;           ///< Torque at a point of the body in body frame (NED)

  double m_forceLimitUser;
  double m_torqueLimitUser;
  Force m_forceInWorldAtCOG_limited;          ///< Force at the COG in world frame with applied limit
  Torque m_torqueInBodyAtCOG_limited;         ///< Torque at the COG in body frame with appliced limit


 public:

  TestFrForce(const std::string &name, const std::string &type_name, FrBody *body) : FrForce(name, type_name, body) {}

  /// Tests
  void TestMaxForceLimit(double fmax);

  void TestMaxTorqueLimit(double fmax);

  void TestSetLimit();

  void TestForceTorqueInWorldAtPointInBody();

  void TestForceTorqueInWorldAtPointInWorld();

  void TestForceTorqueInBodyAtPointInBody();

  void TestForceTorqueInBodyAtPointInWorld();

  void TestForceTorqueInWorldAtCOG();

  void TestForceTorqueInBodyAtCOG();

  void TestTorqueInBodyAtCOG();

  void TestTorqueInWorldAtCOG();

  void TestForceInWorldAtCOG();

  void TestForceInBodyAtCOG();

  void TestForceInWorldAtPointInBody();

  void TestForceInWorldAtPointInWorld();

  void TestForceInBodyAtPointInWorld();

  void TestForceInBodyAtPointInBody();

  void TestGetForceInWorldReference();

  void TestGetForceInWorldComponent();

  void TestGetForceInBody();

  void TestGetForceInBodyReference();

  void TestGetForceInBodyComponent();

  void TestGetTorqueInWorld();

  void TestGetTorqueInWorldReference();

  void TestGetTorqueInWorldComponent();

  void TestGetTorqueInBodyReference();

  void TestGetTorqueInBodyComponent();

  void TestGetTorqueAtPoint();

  void TestForceNorm();

  void TestTorqueNorm();

  void TestGetForceNED();

  void TestGetTorqueNED();

  void TestSpecificCase();

  void TestUpdateLimitApplication();

  /// Accessor
  Position GetPointREFInWorld() const { return m_PointREFInWorld; }

  Position GetPointCOGInBody() const { return m_PointCOGInBody; }

  FrUnitQuaternion GetQuatREF() const { return m_quatREF; }

  /// Methods
  void LoadData(std::string filename);

  Position ReadPosition(FrHDF5Reader &reader, std::string field);

  Force ReadForce(FrHDF5Reader &reader, std::string field);

  Torque ReadTorque(FrHDF5Reader &reader, std::string field);

  Direction ReadDirection(FrHDF5Reader &reader, std::string field);

  void CheckForceInWorldAtCOG();

  void CheckTorqueInBodyAtCOG();

  void CheckForceTorqueAtCOG();

  void CheckLeverArmTorqueInBodyAtCOG();

 private:
  /// Override pure virtual methods
  void Compute(double time) override {};

  void Initialize() override {};
};

Position TestFrForce::ReadPosition(FrHDF5Reader &reader, std::string field) {
  auto value = reader.ReadDoubleArray(field);
  return Position(value(0), value(1), value(2));
}

Force TestFrForce::ReadForce(FrHDF5Reader &reader, std::string field) {
  auto value = reader.ReadDoubleArray(field);
  return Force(value(0), value(1), value(2));
}

Torque TestFrForce::ReadTorque(FrHDF5Reader &reader, std::string field) {
  auto value = reader.ReadDoubleArray(field);
  return Torque(value(0), value(1), value(2));
}

Direction TestFrForce::ReadDirection(FrHDF5Reader &reader, std::string field) {
  auto value = reader.ReadDoubleArray(field);
  return Torque(value(0), value(1), value(2));
}

void TestFrForce::LoadData(std::string filename) {

  FrHDF5Reader reader;

  reader.SetFilename(filename);
  std::string group = "/force_general/";

  m_PointREFInWorld = ReadPosition(reader, group + "PointREFInWorld/");
  m_PointCOGInBody = ReadPosition(reader, group + "PointCOGInBody/");
  m_PointInBody = ReadPosition(reader, group + "PointInBody/");
  m_PointCOGInWorld = ReadPosition(reader, group + "PointCOGInWorld/");
  m_PointInWorld = ReadPosition(reader, group + "PointInWorld/");

  auto direction = ReadDirection(reader, group + "RotationDirection/");
  direction.normalize();
  auto angle = reader.ReadDouble(group + "RotationAngle/");
  m_quatREF = FrUnitQuaternion(direction, angle, NWU);
  m_frameREF = FrFrame(m_PointREFInWorld, m_quatREF, NWU);

  m_forceInWorldAtPoint = ReadForce(reader, group + "ForceInWorldAtPoint/");
  m_forceInBodyAtPoint = ReadForce(reader, group + "ForceInBodyAtPoint/");
  m_forceInWorldAtCOG = ReadForce(reader, group + "ForceInWorldAtCOG/");
  m_forceInBodyAtCOG = ReadForce(reader, group + "ForceInBodyAtCOG/");
  m_torqueInWorldAtPoint = ReadTorque(reader, group + "TorqueInWorldAtPoint/");
  m_torqueInBodyAtPoint = ReadTorque(reader, group + "TorqueInBodyAtPoint/");
  m_torqueInWorldAtCOG = ReadTorque(reader, group + "TorqueInWorldAtCOG/");
  m_torqueInBodyAtCOG = ReadTorque(reader, group + "TorqueInBodyAtCOG/");

  // Create vector in NED convention
  m_PointInBody_NED = m_PointInBody;
  m_PointInWorld_NED = m_PointInWorld;
  m_forceInWorldAtPoint_NED = m_forceInWorldAtPoint;
  m_forceInBodyAtPoint_NED = m_forceInBodyAtPoint;
  m_forceInWorldAtCOG_NED = m_forceInWorldAtCOG;
  m_forceInBodyAtCOG_NED = m_forceInBodyAtCOG;
  m_torqueInWorldAtPoint_NED = m_torqueInWorldAtPoint;
  m_torqueInBodyAtPoint_NED = m_torqueInBodyAtPoint;
  m_torqueInWorldAtCOG_NED = m_torqueInWorldAtCOG;
  m_torqueInBodyAtCOG_NED = m_torqueInBodyAtCOG;
  internal::SwapFrameConvention(m_PointInBody_NED);
  internal::SwapFrameConvention(m_PointInWorld_NED);
  internal::SwapFrameConvention(m_forceInWorldAtPoint_NED);
  internal::SwapFrameConvention(m_forceInBodyAtPoint_NED);
  internal::SwapFrameConvention(m_forceInWorldAtCOG_NED);
  internal::SwapFrameConvention(m_forceInBodyAtCOG_NED);
  internal::SwapFrameConvention(m_torqueInWorldAtPoint_NED);
  internal::SwapFrameConvention(m_torqueInBodyAtPoint_NED);
  internal::SwapFrameConvention(m_torqueInWorldAtCOG_NED);
  internal::SwapFrameConvention(m_torqueInBodyAtCOG_NED);

  m_forceLimitUser = reader.ReadDouble(group + "ForceLimit/");
  m_torqueLimitUser = reader.ReadDouble(group + "TorqueLimit/");
  m_forceInWorldAtCOG_limited = ReadForce(reader, group + "ForceInWorldAtCOG_limited/");
  m_torqueInBodyAtCOG_limited = ReadTorque(reader, group + "TorqueInBodyAtCOG_limited/");
}

void TestFrForce::CheckForceInWorldAtCOG() {
  GetBody()->Update();
  auto force = this->GetForceInWorld(NWU);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());
}

void TestFrForce::CheckTorqueInBodyAtCOG() {
  GetBody()->Update();
  auto torque = this->GetTorqueInBodyAtCOG(NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());
}

void TestFrForce::CheckForceTorqueAtCOG() {
  this->CheckForceInWorldAtCOG();
  this->CheckTorqueInBodyAtCOG();
}

void TestFrForce::CheckLeverArmTorqueInBodyAtCOG() {
  GetBody()->Update();
  auto torque = this->GetTorqueInBodyAtCOG(NWU);
  Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

  EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());
}


void TestFrForce::TestMaxForceLimit(double fmax) {
  this->SetMaxForceLimit(fmax);
  EXPECT_FLOAT_EQ(this->GetMaxForceLimit(), fmax);
}

void TestFrForce::TestMaxTorqueLimit(double fmax) {
  this->SetMaxTorqueLimit(fmax);
  EXPECT_FLOAT_EQ(this->GetMaxTorqueLimit(), fmax);
}

void TestFrForce::TestSetLimit() {
  this->SetLimit(true);
  EXPECT_TRUE(this->GetLimit());
}

void TestFrForce::TestForceTorqueInWorldAtPointInBody() {
  this->SetForceTorqueInWorldAtPointInBody(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInBody, NWU);
  CheckForceTorqueAtCOG();
  this->SetForceTorqueInWorldAtPointInBody(m_forceInWorldAtPoint_NED, m_torqueInWorldAtPoint_NED, m_PointInBody_NED,
                                           NED);
  CheckForceTorqueAtCOG();
}

void TestFrForce::TestForceTorqueInBodyAtPointInBody() {
  this->SetForceTorqueInBodyAtPointInBody(m_forceInBodyAtPoint, m_torqueInBodyAtPoint, m_PointInBody, NWU);
  CheckForceTorqueAtCOG();
  this->SetForceTorqueInBodyAtPointInBody(m_forceInBodyAtPoint_NED, m_torqueInBodyAtPoint_NED, m_PointInBody_NED, NED);
  CheckForceTorqueAtCOG();
}

void TestFrForce::TestForceTorqueInWorldAtPointInWorld() {
  this->SetForceTorqueInWorldAtPointInWorld(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInWorld, NWU);
  CheckForceTorqueAtCOG();
  this->SetForceTorqueInWorldAtPointInWorld(m_forceInWorldAtPoint_NED, m_torqueInWorldAtPoint_NED, m_PointInWorld_NED,
                                            NED);
  CheckForceTorqueAtCOG();
}

void TestFrForce::TestForceTorqueInBodyAtPointInWorld() {
  this->SetForceTorqueInBodyAtPointInWorld(m_forceInBodyAtPoint, m_torqueInBodyAtPoint, m_PointInWorld, NWU);
  CheckForceTorqueAtCOG();
  this->SetForceTorqueInBodyAtPointInWorld(m_forceInBodyAtPoint_NED, m_torqueInBodyAtPoint_NED, m_PointInWorld_NED,
                                           NED);
  CheckForceTorqueAtCOG();
}

void TestFrForce::TestForceTorqueInWorldAtCOG() {
  this->SetForceTorqueInWorldAtCOG(m_forceInWorldAtCOG, m_torqueInWorldAtCOG, NWU);
  CheckForceTorqueAtCOG();
  this->SetForceTorqueInWorldAtCOG(m_forceInWorldAtCOG_NED, m_torqueInWorldAtCOG_NED, NED);
  CheckForceTorqueAtCOG();
}

void TestFrForce::TestForceTorqueInBodyAtCOG() {
  this->SetForceTorqueInBodyAtCOG(m_forceInBodyAtCOG, m_torqueInBodyAtCOG, NWU);
  CheckForceTorqueAtCOG();
  this->SetForceTorqueInBodyAtCOG(m_forceInBodyAtCOG_NED, m_torqueInBodyAtCOG_NED, NED);
  CheckForceTorqueAtCOG();
}

void TestFrForce::TestTorqueInBodyAtCOG() {
  this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
  CheckTorqueInBodyAtCOG();
  this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG_NED, NED);
  CheckTorqueInBodyAtCOG();
}

void TestFrForce::TestTorqueInWorldAtCOG() {
  this->SetTorqueInWorldAtCOG(m_torqueInWorldAtCOG, NWU);
  CheckTorqueInBodyAtCOG();
  this->SetTorqueInWorldAtCOG(m_torqueInWorldAtCOG_NED, NED);
  CheckTorqueInBodyAtCOG();
}

void TestFrForce::TestForceInBodyAtCOG() {
  this->SetForceInBody(m_forceInBodyAtCOG, NWU);
  CheckForceInWorldAtCOG();
  this->SetForceInBody(m_forceInBodyAtCOG_NED, NED);
  CheckForceInWorldAtCOG();
}

void TestFrForce::TestForceInWorldAtCOG() {
  this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);
  CheckForceInWorldAtCOG();
  this->SetForceInWorldAtCOG(m_forceInWorldAtCOG_NED, NED);
  CheckForceInWorldAtCOG();
}

void TestFrForce::TestForceInWorldAtPointInBody() {
  this->SetForceInWorldAtPointInBody(m_forceInWorldAtPoint, m_PointInBody, NWU);
  CheckForceInWorldAtCOG();
  CheckLeverArmTorqueInBodyAtCOG();
  this->SetForceInWorldAtPointInBody(m_forceInWorldAtPoint_NED, m_PointInBody_NED, NED);
  CheckForceInWorldAtCOG();
  CheckLeverArmTorqueInBodyAtCOG();
}

void TestFrForce::TestForceInWorldAtPointInWorld() {
  this->SetForceInWorldAtPointInWorld(m_forceInWorldAtPoint, m_PointInWorld, NWU);
  CheckForceInWorldAtCOG();
  CheckLeverArmTorqueInBodyAtCOG();
  this->SetForceInWorldAtPointInWorld(m_forceInWorldAtPoint_NED, m_PointInWorld_NED, NED);
  CheckForceInWorldAtCOG();
  CheckLeverArmTorqueInBodyAtCOG();
}

void TestFrForce::TestForceInBodyAtPointInBody() {
  this->SetForceInBodyAtPointInBody(m_forceInBodyAtPoint, m_PointInBody, NWU);
  CheckForceInWorldAtCOG();
  CheckLeverArmTorqueInBodyAtCOG();
  this->SetForceInBodyAtPointInBody(m_forceInBodyAtPoint_NED, m_PointInBody_NED, NED);
  CheckForceInWorldAtCOG();
  CheckLeverArmTorqueInBodyAtCOG();
}

void TestFrForce::TestForceInBodyAtPointInWorld() {
  this->SetForceInBodyAtPointInWorld(m_forceInBodyAtPoint, m_PointInWorld, NWU);
  CheckForceInWorldAtCOG();
  CheckLeverArmTorqueInBodyAtCOG();
  this->SetForceInBodyAtPointInWorld(m_forceInBodyAtPoint_NED, m_PointInWorld_NED, NED);
  CheckForceInWorldAtCOG();
  CheckLeverArmTorqueInBodyAtCOG();
}

void TestFrForce::TestGetForceInWorldReference() {

  this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

  GetBody()->Update();

  Force force;
  GetForceInWorld(force, NWU);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

  GetForceInWorld(force, NED);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG_NED.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG_NED.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG_NED.GetFz());
}

void TestFrForce::TestGetForceInWorldComponent() {

  this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

  GetBody()->Update();

  Force force;
  GetForceInWorld(force.GetFx(), force.GetFy(), force.GetFz(), NWU);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

  GetForceInWorld(force.GetFx(), force.GetFy(), force.GetFz(), NED);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG_NED.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG_NED.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG_NED.GetFz());
}

void TestFrForce::TestGetForceInBody() {

  this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

  GetBody()->Update();

  auto force = GetForceInBody(NWU);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

  force = GetForceInBody(NED);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG_NED.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG_NED.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG_NED.GetFz());
}

void TestFrForce::TestGetForceInBodyReference() {

  this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

  GetBody()->Update();

  Force force;
  GetForceInBody(force, NWU);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

  GetForceInBody(force, NED);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG_NED.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG_NED.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG_NED.GetFz());
}


void TestFrForce::TestGetForceInBodyComponent() {

  this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);
  GetBody()->Update();

  Force force;
  GetForceInBody(force.GetFx(), force.GetFy(), force.GetFz(), NWU);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

  GetForceInBody(force.GetFx(), force.GetFy(), force.GetFz(), NED);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG_NED.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG_NED.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG_NED.GetFz());
}


void TestFrForce::TestGetTorqueInWorld() {

  this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
  GetBody()->Update();

  auto torque = GetTorqueInWorldAtCOG(NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG.GetMz());

  torque = GetTorqueInWorldAtCOG(NED);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG_NED.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG_NED.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG_NED.GetMz());
}

void TestFrForce::TestGetTorqueInWorldReference() {

  this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
  GetBody()->Update();

  Torque torque;
  GetTorqueInWorldAtCOG(torque, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG.GetMz());

  GetTorqueInWorldAtCOG(torque, NED);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG_NED.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG_NED.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG_NED.GetMz());
}

void TestFrForce::TestGetTorqueInWorldComponent() {

  this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
  GetBody()->Update();

  Torque torque;
  GetTorqueInWorldAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG.GetMz());

  GetTorqueInWorldAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NED);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG_NED.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG_NED.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG_NED.GetMz());
}

void TestFrForce::TestGetTorqueInBodyReference() {

  this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
  GetBody()->Update();

  Torque torque;
  GetTorqueInBodyAtCOG(torque, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

  GetTorqueInBodyAtCOG(torque, NED);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG_NED.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG_NED.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG_NED.GetMz());
}

void TestFrForce::TestGetTorqueInBodyComponent() {

  this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
  GetBody()->Update();

  Torque torque;
  GetTorqueInBodyAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

  GetTorqueInBodyAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NED);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG_NED.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG_NED.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG_NED.GetMz());
}

void TestFrForce::TestGetTorqueAtPoint() {

  SetForceTorqueInWorldAtPointInWorld(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInWorld, NWU);
  GetBody()->Update();

  Torque torque;
  GetTorqueInWorldAtPointInWorld(torque, m_PointInWorld, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtPoint.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtPoint.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtPoint.GetMz());

  torque = GetTorqueInWorldAtPointInWorld(m_PointInWorld, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtPoint.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtPoint.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtPoint.GetMz());

  GetTorqueInWorldAtPointInBody(torque, m_PointInBody, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtPoint.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtPoint.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtPoint.GetMz());

  torque = GetTorqueInWorldAtPointInBody(m_PointInBody, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtPoint.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtPoint.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtPoint.GetMz());

  GetTorqueInBodyAtPointInWorld(torque, m_PointInWorld, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtPoint.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtPoint.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtPoint.GetMz());

  torque = GetTorqueInBodyAtPointInWorld(m_PointInWorld, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtPoint.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtPoint.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtPoint.GetMz());

  GetTorqueInBodyAtPointInBody(torque, m_PointInBody, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtPoint.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtPoint.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtPoint.GetMz());

  torque = GetTorqueInBodyAtPointInBody(m_PointInBody, NWU);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtPoint.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtPoint.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtPoint.GetMz());

}

void TestFrForce::TestForceNorm() {

  this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);
  GetBody()->Update();

  double normREF = std::sqrt(m_forceInWorldAtCOG.GetFx() * m_forceInWorldAtCOG.GetFx()
                             + m_forceInWorldAtCOG.GetFy() * m_forceInWorldAtCOG.GetFy()
                             + m_forceInWorldAtCOG.GetFz() * m_forceInWorldAtCOG.GetFz());

  auto normTEST = GetForceNorm();

  EXPECT_FLOAT_EQ(normTEST, normREF);
}


void TestFrForce::TestTorqueNorm() {

  this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
  GetBody()->Update();

  double normREF = std::sqrt(m_torqueInBodyAtCOG.GetMx() * m_torqueInBodyAtCOG.GetMx()
                             + m_torqueInBodyAtCOG.GetMy() * m_torqueInBodyAtCOG.GetMy()
                             + m_torqueInBodyAtCOG.GetMz() * m_torqueInBodyAtCOG.GetMz());

  auto normTEST = GetTorqueNormAtCOG();

  EXPECT_FLOAT_EQ(normTEST, normREF);
}

void TestFrForce::TestGetForceNED() {

  this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

  auto force = GetForceInWorld(NED);
  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
  EXPECT_FLOAT_EQ(-force.GetFy(), m_forceInWorldAtCOG.GetFy());
  EXPECT_FLOAT_EQ(-force.GetFz(), m_forceInWorldAtCOG.GetFz());
}

void TestFrForce::TestGetTorqueNED() {

  this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);

  auto torque = GetTorqueInBodyAtCOG(NED);
  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
  EXPECT_FLOAT_EQ(-torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
  EXPECT_FLOAT_EQ(-torque.GetMz(), m_torqueInBodyAtCOG.GetMz());
}

void TestFrForce::TestSpecificCase() {
  this->SetForceTorqueInBodyAtPointInBody(m_forceInBodyAtCOG, m_torqueInBodyAtCOG, m_PointCOGInBody, NWU);
  CheckForceInWorldAtCOG();
  CheckTorqueInBodyAtCOG();
  this->SetForceTorqueInWorldAtPointInWorld(m_forceInWorldAtCOG, m_torqueInWorldAtCOG, m_PointCOGInWorld, NWU);
  CheckForceInWorldAtCOG();
  CheckTorqueInBodyAtCOG();
}

void TestFrForce::TestUpdateLimitApplication() {

  this->SetMaxForceLimit(m_forceLimitUser);
  this->SetMaxTorqueLimit(m_torqueLimitUser);
  this->SetLimit(true);

  this->SetForceTorqueInBodyAtCOG(m_forceInBodyAtCOG, m_torqueInBodyAtCOG, NWU);
  m_chronoForce->UpdateState();

  auto force = GetForceInWorld(NWU);
  auto torque = GetTorqueInBodyAtCOG(NWU);

  EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG_limited.GetFx());
  EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG_limited.GetFy());
  EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG_limited.GetFz());

  EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG_limited.GetMx());
  EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG_limited.GetMy());
  EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG_limited.GetMz());
};

//
// TESTING CLASS FUNCTION
//

class TestBase : public ::testing::Test {

 public:

  TestBase() : system("test_FrForce") {}

  std::shared_ptr<FrBody> GetBody() const {return body;}

 protected:

  /// Environment objects
  FrOffshoreSystem system;
  std::shared_ptr<FrBody> body;
  std::shared_ptr<TestFrForce> force;

  /// Initialize environment
  void SetUp() override;

};

void TestBase::SetUp() {

  body = system.NewBody("body");

  force = std::make_shared<TestFrForce>("test_FrForce", "TestFrForce", body.get());
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  force->LoadData(database);

  body->SetPosition(force->GetPointREFInWorld(), NWU);
  FrInertiaTensor InertiaTensor(1., 1., 1., 1., 0., 0., 0., force->GetPointCOGInBody(), NWU);
  body->SetInertiaTensor(InertiaTensor);

  body->SetRotation(force->GetQuatREF());
  body->AddExternalForce(force);
  body->Update();
}

//
// LIST OF GTEST
//

TEST_F(TestBase, MaxForceLimit) {
  force->TestMaxForceLimit(5.e6);
}

TEST_F(TestBase, MaxTorqueLimit) {
  force->TestMaxTorqueLimit(9.e9);
}

TEST_F(TestBase, SetLimit) {
  force->TestSetLimit();
}

TEST_F(TestBase, ForceTorqueInWorldAtPointInBody) {
  force->TestForceTorqueInWorldAtPointInBody();
}

TEST_F(TestBase, ForceTorqueInBodyAtPointInBody) {
  force->TestForceTorqueInBodyAtPointInBody();
}

TEST_F(TestBase, ForceTorqueInWorldAtPointInWorld) {
  force->TestForceTorqueInWorldAtPointInWorld();
}

TEST_F(TestBase, ForceTorqueInBodyAtPointInWorld) {
  force->TestForceTorqueInBodyAtPointInWorld();
}

TEST_F(TestBase, ForceTorqueInWorldAtCOG) {
  force->TestForceTorqueInWorldAtCOG();
}

TEST_F(TestBase, ForceTorqueInBodyAtCOG) {
  force->TestForceTorqueInBodyAtCOG();
}

TEST_F(TestBase, TorqueInBodyAtCOG) {
  force->TestTorqueInBodyAtCOG();
}

TEST_F(TestBase, TorqueInWorldAtCOG) {
  force->TestTorqueInWorldAtCOG();
}

TEST_F(TestBase, ForceInBodyAtCOG) {
  force->TestForceInBodyAtCOG();
}

TEST_F(TestBase, ForceInWorldAtCOG) {
  force->TestForceInWorldAtCOG();
}

TEST_F(TestBase, ForceInWorldAtPointInBody) {
  force->TestForceInWorldAtPointInBody();
}

TEST_F(TestBase, ForceInWorldAtPointInWorld) {
  force->TestForceInWorldAtPointInWorld();
}

TEST_F(TestBase, ForceInBodyAtPointInBody) {
  force->TestForceInBodyAtPointInBody();
}

TEST_F(TestBase, ForceInBodyAtPointInWorld) {
  force->TestForceInBodyAtPointInWorld();
}

TEST_F(TestBase, GetForceInWorldReference) {
  force->TestGetForceInWorldReference();
}

TEST_F(TestBase, GetForceInWorldComponent) {
  force->TestGetForceInWorldComponent();
}

TEST_F(TestBase, GetForceInBody) {
  force->TestGetForceInBody();
}

TEST_F(TestBase, GetForceInBodyReference) {
  force->TestGetForceInBodyReference();
}

TEST_F(TestBase, GetForceInBodyComponent) {
  force->TestGetForceInBodyComponent();
}

TEST_F(TestBase, GetTorqueInWorld) {
  force->TestGetTorqueInWorld();
}

TEST_F(TestBase, GetTorqueInWorldReference) {
  force->TestGetTorqueInWorldReference();
}

TEST_F(TestBase, GetTorqueInBodyReference) {
  force->TestGetTorqueInBodyReference();
}

TEST_F(TestBase, GetTorqueInBodyComponent) {
  force->TestGetTorqueInBodyComponent();
}

TEST_F(TestBase, GetTorqueInWorldComponent) {
  force->TestGetTorqueInWorldComponent();
}

TEST_F(TestBase, GetTorqueAtPoint) {
  force->TestGetTorqueAtPoint();
}

TEST_F(TestBase, ForceNorm) {
  force->TestForceNorm();
}

TEST_F(TestBase, TorqueNorm) {
  force->TestTorqueNorm();
}

TEST_F(TestBase, GetForceNED) {
  force->TestGetForceNED();
}

TEST_F(TestBase, GetTorqueNED) {
  force->TestGetTorqueNED();
}

TEST_F(TestBase, SpecificCase) {
  force->TestSpecificCase();
}

TEST_F(TestBase, UpdateLimitApplication) {
  force->TestUpdateLimitApplication();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
