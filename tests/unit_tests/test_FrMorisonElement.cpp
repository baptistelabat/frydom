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

// ---------------------------------------------------------------
//
// MORISON SINGLE ELEMENT
//
// ---------------------------------------------------------------

class TestMorisonSingleElement : public FrMorisonSingleElement {

 public:
  TestMorisonSingleElement(FrBody *body) : FrMorisonSingleElement(body) {}

  TestMorisonSingleElement(FrBody *body, Position posA, Position posB, double diameter,
                           MorisonCoeff ca, MorisonCoeff cd, double cf,
                           Direction perpendicular = Direction(0., 0., 1.));

  TestMorisonSingleElement(std::shared_ptr<FrNode> nodeA, std::shared_ptr<FrNode> nodeB,
                           double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                           Direction perpendicular = Direction(0., 0., 1.));

  void TestAddedMass(double ca);

  void TestAddedMass(double caX, double caY);

  void TestDragCoeff(double cd);

  void TestDragCoeff(double cdX, double cdY);

  void TestFriction(double cf);

  void CheckDiameter(double d);

  void CheckNodesPointer(std::shared_ptr<FrNode> nodeA, std::shared_ptr<FrNode> nodeB);

  void CheckNodesPointer(FrBody *body, Position posA, Position posB);

  void CheckLength(double l);

  void CheckAddedMass(double caX, double caY);

  void CheckDragCoeff(double cdX, double cdY);

  void CheckFriction(double cf);
};

TestMorisonSingleElement::TestMorisonSingleElement(FrBody *body, Position posA, Position posB, double diameter,
                                                   MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                   Direction perpendicular)
    : FrMorisonSingleElement(body, posA, posB, diameter, ca, cd, cf, perpendicular) {

  CheckDiameter(diameter);
  CheckAddedMass(ca.x, ca.y);
  CheckDragCoeff(cd.x, cd.y);
  CheckFriction(cf);
  CheckLength((posB - posA).norm());
}

TestMorisonSingleElement::TestMorisonSingleElement(std::shared_ptr<FrNode> nodeA, std::shared_ptr<FrNode> nodeB,
                                                   double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                   Direction perpendicular)
    : FrMorisonSingleElement(nodeA, nodeB, diameter, ca, cd, cf, perpendicular) {

  CheckNodesPointer(nodeA, nodeB);
  CheckDiameter(diameter);
  CheckAddedMass(ca.x, ca.y);
  CheckDragCoeff(cd.x, cd.y);
  CheckFriction(cf);
  CheckLength((nodeB->GetPositionInWorld(NWU) - nodeA->GetPositionInWorld(NWU)).norm());
}


void TestMorisonSingleElement::TestAddedMass(double ca) {
  SetAddedMass(ca);
  EXPECT_FLOAT_EQ(ca, m_property.ca.x);
  EXPECT_FLOAT_EQ(ca, m_property.ca.y);
}

void TestMorisonSingleElement::TestAddedMass(double caX, double caY) {
  SetAddedMass({caX, caY});
  EXPECT_FLOAT_EQ(caX, m_property.ca.x);
  EXPECT_FLOAT_EQ(caY, m_property.ca.y);
}

void TestMorisonSingleElement::TestDragCoeff(double cd) {
  SetDragCoeff(cd);
  EXPECT_FLOAT_EQ(cd, m_property.cd.x);
  EXPECT_FLOAT_EQ(cd, m_property.cd.y);
}

void TestMorisonSingleElement::TestDragCoeff(double cdX, double cdY) {
  SetDragCoeff({cdX, cdY});
  EXPECT_FLOAT_EQ(cdX, m_property.cd.x);
  EXPECT_FLOAT_EQ(cdY, m_property.cd.y);
}

void TestMorisonSingleElement::TestFriction(double cf) {
  SetFrictionCoeff(cf);
  EXPECT_FLOAT_EQ(cf, m_property.cf);
}

void TestMorisonSingleElement::CheckDiameter(double d) {
  EXPECT_FLOAT_EQ(d, m_property.diameter);
}

void TestMorisonSingleElement::CheckNodesPointer(std::shared_ptr<FrNode> nodeA, std::shared_ptr<FrNode> nodeB) {
  EXPECT_EQ(m_nodeA, nodeA);
  EXPECT_EQ(m_nodeB, nodeB);
}

void TestMorisonSingleElement::CheckNodesPointer(FrBody *body, Position posA, Position posB) {
  EXPECT_EQ(m_nodeA->GetPositionInWorld(NWU), posA);
  EXPECT_EQ(m_nodeB->GetPositionInWorld(NWU), posB);
  EXPECT_EQ(m_nodeA->GetBody(), body);
  EXPECT_EQ(m_nodeB->GetBody(), body);
}

void TestMorisonSingleElement::CheckLength(double l) {
  EXPECT_FLOAT_EQ(m_property.length, l);
}

void TestMorisonSingleElement::CheckAddedMass(double caX, double caY) {
  EXPECT_FLOAT_EQ(caX, m_property.ca.x);
  EXPECT_FLOAT_EQ(caY, m_property.ca.y);
}

void TestMorisonSingleElement::CheckDragCoeff(double cdX, double cdY) {
  EXPECT_FLOAT_EQ(cdX, m_property.cd.x);
  EXPECT_FLOAT_EQ(cdY, m_property.cd.y);
}

void TestMorisonSingleElement::CheckFriction(double cf) {
  EXPECT_FLOAT_EQ(cf, m_property.cf);
}

// ---------------------------------------------------------------
//
// MORISON COMPOSITE ELEMENT
//
// ---------------------------------------------------------------

class TestMorisonCompositeElement : public FrMorisonCompositeElement {


 public:
  TestMorisonCompositeElement(FrBody *body) : FrMorisonCompositeElement(body) {}

  void TestDragCoeff(double cdX, double cdY);

  void TestAddedMass(double caX, double caY);

  void TestFrictionCoeff(double cf);

  void TestDiameter(double diameter);
};

void TestMorisonCompositeElement::TestDragCoeff(double cdX, double cdY) {
  SetDragCoeff({cdX, cdY});
  EXPECT_FLOAT_EQ(cdX, m_property.cd.x);
  EXPECT_FLOAT_EQ(cdY, m_property.cd.y);
}

void TestMorisonCompositeElement::TestAddedMass(double caX, double caY) {
  SetAddedMass({caX, caY});
  EXPECT_FLOAT_EQ(caX, m_property.ca.x);
  EXPECT_FLOAT_EQ(caY, m_property.ca.y);
}

void TestMorisonCompositeElement::TestFrictionCoeff(double cf) {
  SetFrictionCoeff(cf);
  EXPECT_FLOAT_EQ(cf, m_property.cf);
}

void TestMorisonCompositeElement::TestDiameter(double diameter) {
  SetDiameter(diameter);
  EXPECT_FLOAT_EQ(diameter, m_property.diameter);
}

// -----------------------------------------------------------------
//
// MORISON TEST
//
// -----------------------------------------------------------------

class TestMorison : public ::testing::Test {

 public:
  FrOffshoreSystem system;
  std::shared_ptr<FrBody> body;

 protected:
  Velocity m_flowVelocityInFrame;
  Velocity m_flowVelocityInWorld;
  Force m_MorisonForce;
  Torque m_MorisonTorque;
  Position m_pointA;
  Position m_pointB;
  MorisonCoeff m_addedMass;
  MorisonCoeff m_dragCoeff;
  double m_frictionCoeff;
  double m_diameter;

 protected:

  TestMorison() : system("TestMorison") {}

  void SetUp() override;

  void LoadData(std::string filename);

  template<class Vector>
  Vector ReadVector(FrHDF5Reader &reader, std::string field) const;

  MorisonCoeff ReadMorisonCoeff(FrHDF5Reader &reader, std::string field) const;

  void CheckForce(FrForce *force) const;
};

template<class Vector>
Vector TestMorison::ReadVector(FrHDF5Reader &reader, std::string field) const {
  auto value = reader.ReadDoubleArray(field);
  return Vector(value(0), value(1), value(2));
}

MorisonCoeff TestMorison::ReadMorisonCoeff(FrHDF5Reader &reader, std::string field) const {
  auto value = reader.ReadDoubleArray(field);
  return MorisonCoeff(value(0), value(1));
}

void TestMorison::SetUp() {
//    body = std::make_shared<FrBody>();
//    system.AddBody(body);
  body = system.NewBody("body");
  body->SetFixedInWorld(true);
//    system.GetEnvironment()->GetOcean()->SetInfiniteDepth();
  system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(-41.38, NWU);
}

void TestMorison::LoadData(std::string filename) {

  FrHDF5Reader reader;

  reader.SetFilename(filename);
  std::string group = "/morison/";

  auto waveHeight = reader.ReadDouble(group + "WaveHeight");
  auto wavePeriod = reader.ReadDouble(group + "WavePeriod");

  auto waveField = system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWaveHeight(waveHeight);
  waveField->SetWavePeriod(wavePeriod);

  auto pointRef = ReadVector<Position>(reader, group + "PointRef");
  body->SetPosition(pointRef, NWU);

  auto posCOG = ReadVector<Position>(reader, group + "COG");

  FrInertiaTensor InertiaTensor(1., 1., 1., 1., 0., 0., 0., posCOG, NWU);
  body->SetInertiaTensor(InertiaTensor);

  auto direction = ReadVector<Direction>(reader, group + "RotationDirection");
  direction.normalize();
  auto angle = reader.ReadDouble(group + "RotationAngle");
  body->SetRotation(FrUnitQuaternion(direction, angle, NWU));

  auto bodyVelocity = ReadVector<Velocity>(reader, group + "BodyVelocity");
  auto bodyAngularVelocity = ReadVector<Velocity>(reader, group + "BodyAngularVelocity");
  body->SetGeneralizedVelocityInWorld(bodyVelocity, bodyAngularVelocity, NWU);

  m_pointA = ReadVector<Position>(reader, group + "PointA");
  m_pointB = ReadVector<Position>(reader, group + "PointB");

  m_addedMass = ReadMorisonCoeff(reader, group + "AddedMass");
  m_dragCoeff = ReadMorisonCoeff(reader, group + "DragCoeff");
  m_frictionCoeff = reader.ReadDouble(group + "FrictionCoeff");
  m_diameter = reader.ReadDouble(group + "Diameter");

  m_MorisonForce = ReadVector<Force>(reader, group + "MorisonForce");
  m_MorisonTorque = ReadVector<Torque>(reader, group + "MorisonTorque");
}

void TestMorison::CheckForce(FrForce *force) const {

  auto forceWorld = force->GetForceInWorld(NWU);
  auto torqueBody = force->GetTorqueInBodyAtCOG(NWU);

  EXPECT_NEAR(m_MorisonForce.GetFx(), forceWorld.GetFx(), 10e-8);
  EXPECT_NEAR(m_MorisonForce.GetFy(), forceWorld.GetFy(), 10e-8);
  EXPECT_NEAR(m_MorisonForce.GetFz(), forceWorld.GetFz(), 10e-8);

  EXPECT_NEAR(m_MorisonTorque.GetMx(), torqueBody.GetMx(), 10e-8);
  EXPECT_NEAR(m_MorisonTorque.GetMy(), torqueBody.GetMy(), 10e-8);
  EXPECT_NEAR(m_MorisonTorque.GetMz(), torqueBody.GetMz(), 10e-8);

}

// -----------------------------------------------------------------
//
// GTEST
//
// -----------------------------------------------------------------

TEST_F(TestMorison, SetAddedMass) {
  auto morison = std::make_shared<TestMorisonSingleElement>(body.get());
  morison->TestAddedMass(1.);
  morison->TestAddedMass(1., 2.);
}

TEST_F(TestMorison, SetDragCoeff) {
  auto morison = std::make_shared<TestMorisonSingleElement>(body.get());
  morison->TestDragCoeff(0.1);
  morison->TestDragCoeff(0.2, 0.5);
}

TEST_F(TestMorison, SetFriction) {
  auto morison = std::make_shared<TestMorisonSingleElement>(body.get());
  morison->TestFriction(0.1);
}

TEST_F(TestMorison, SetDiameter) {
  auto morison = std::make_shared<TestMorisonSingleElement>(body.get());
  morison->SetDiameter(0.5);
  morison->CheckDiameter(0.5);
  EXPECT_FLOAT_EQ(0.5, morison->GetDiameter());
}

TEST_F(TestMorison, TestNodes) {

  auto morison = std::make_shared<TestMorisonSingleElement>(body.get());

  Position posA = Position(1., -2., -5.);
  Position posB = Position(1.2, 4.5, -3.);
  double length = (posB - posA).norm();
  auto nodeA = body->NewNode("nodeA");
  nodeA->SetPositionInBody(posA, NWU);
  auto nodeB = body->NewNode("nodeB");
  nodeB->SetPositionInBody(posB, NWU);

  morison->SetNodes(nodeA, nodeB);
  morison->CheckNodesPointer(nodeA, nodeB);

  morison->SetNodes(body.get(), posA, posB);
  morison->CheckNodesPointer(body.get(), posA, posB);
  morison->CheckLength(length);
}

TEST_F(TestMorison, SingleElementConstructor) {

  Position posA = Position(1., -2., -5.);
  Position posB = Position(1.2, 4.5, -3.);
  auto morison = std::make_shared<TestMorisonSingleElement>(body.get(), posA, posB, 0.5, 0.1, 0.2, 0.01);

  auto nodeA = body->NewNode("nodeA");
  nodeA->SetPositionInBody(posA, NWU);
  auto nodeB = body->NewNode("nodeB");
  nodeB->SetPositionInBody(posB, NWU);
  auto morison2 = std::make_shared<TestMorisonSingleElement>(nodeA, nodeB, 0.5, 0.1, 0.2, 0.01);
}

TEST_F(TestMorison, SingleElementForce) {
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);

  auto morison = std::make_shared<TestMorisonSingleElement>(body.get(), m_pointA, m_pointB, m_diameter, m_addedMass,
                                                            m_dragCoeff,
                                                            m_frictionCoeff);

  auto force = make_morison_force("Morison", body, morison);
  system.Initialize();

  force->Update(0.);
  CheckForce(force.get());
}

TEST_F(TestMorison, SingleElementForceWithNode) {
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);

  auto nodeA = body->NewNode("nodeA");
  nodeA->SetPositionInBody(m_pointA, NWU);
  auto nodeB = body->NewNode("nodeB");
  nodeB->SetPositionInBody(m_pointB, NWU);

  auto morison = std::make_shared<TestMorisonSingleElement>(nodeA, nodeB, m_diameter, m_addedMass, m_dragCoeff,
                                                            m_frictionCoeff);

  auto force = make_morison_force("Morison", body, morison);
  system.Initialize();

  force->Update(0.);
  CheckForce(force.get());
}

TEST_F(TestMorison, CompositeElementWithPositions) {
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);

  auto morison = std::make_shared<FrMorisonCompositeElement>(body.get());

  morison->AddElement(m_pointA, m_pointB, m_diameter, m_addedMass, m_dragCoeff, m_frictionCoeff);

  auto force = make_morison_force("Morison", body, morison);
  system.Initialize();

  force->Update(0.);
  CheckForce(force.get());
}

TEST_F(TestMorison, CompositeElementWithNodes) {
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);

  auto morison = std::make_shared<FrMorisonCompositeElement>(body.get());

  auto nodeA = body->NewNode("nodeA");
  nodeA->SetPositionInBody(m_pointA, NWU);
  auto nodeB = body->NewNode("nodeB");
  nodeB->SetPositionInBody(m_pointB, NWU);

  morison->AddElement(nodeA, nodeB, m_diameter, m_addedMass, m_dragCoeff, m_frictionCoeff);

  auto force = make_morison_force("Morison", body, morison);
  system.Initialize();

  force->Update(0.);
  CheckForce(force.get());
}

TEST_F(TestMorison, ElementDiscretization) {
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);

  auto morison = std::make_shared<FrMorisonCompositeElement>(body.get());

  morison->AddElement(m_pointA, m_pointB, m_diameter, m_addedMass, m_dragCoeff, m_frictionCoeff, 10);

  auto force = make_morison_force("Morison", body, morison);
  system.Initialize();

  force->Update(0.);
  CheckForce(force.get());
}

TEST_F(TestMorison, TwoElements) {
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);

  auto morison = std::make_shared<FrMorisonCompositeElement>(body.get());

  morison->AddElement(m_pointA, m_pointB, m_diameter, m_addedMass, m_dragCoeff, m_frictionCoeff);
  morison->AddElement(m_pointA, m_pointB, m_diameter, m_addedMass, m_dragCoeff, m_frictionCoeff);

  auto force = make_morison_force("Morison", body, morison);
  system.Initialize();

  force->Update(0.);

  auto forceWorld = force->GetForceInWorld(NWU);
  auto torqueBody = force->GetTorqueInBodyAtCOG(NWU);

  EXPECT_NEAR(2. * m_MorisonForce.GetFx(), forceWorld.GetFx(), 10e-8);
  EXPECT_NEAR(2. * m_MorisonForce.GetFy(), forceWorld.GetFy(), 10e-8);
  EXPECT_NEAR(2. * m_MorisonForce.GetFz(), forceWorld.GetFz(), 10e-8);

  EXPECT_NEAR(2. * m_MorisonTorque.GetMx(), torqueBody.GetMx(), 10e-8);
  EXPECT_NEAR(2. * m_MorisonTorque.GetMy(), torqueBody.GetMy(), 10e-8);
  EXPECT_NEAR(2. * m_MorisonTorque.GetMz(), torqueBody.GetMz(), 10e-8);
}

TEST_F(TestMorison, CompositeDragCoeff) {
  auto morison = std::make_shared<TestMorisonCompositeElement>(body.get());
  morison->TestDragCoeff(0.1, 0.15);
  auto cd = morison->GetDragCoeff();
  EXPECT_FLOAT_EQ(0.1, cd.x);
  EXPECT_FLOAT_EQ(0.15, cd.y);
}

TEST_F(TestMorison, CompositeAddedMass) {
  auto morison = std::make_shared<TestMorisonCompositeElement>(body.get());
  morison->TestAddedMass(1.5, 2.);
  auto ca = morison->GetAddedMass();
  EXPECT_FLOAT_EQ(1.5, ca.x);
  EXPECT_FLOAT_EQ(2., ca.y);
}

TEST_F(TestMorison, CompositeFrictionCoeff) {
  auto morison = std::make_shared<TestMorisonCompositeElement>(body.get());
  morison->TestFrictionCoeff(0.01);
  auto cf = morison->GetFrictionCoeff();
  EXPECT_FLOAT_EQ(0.01, cf);
}

TEST_F(TestMorison, CompositeDiameter) {
  auto morison = std::make_shared<TestMorisonCompositeElement>(body.get());
  morison->TestDiameter(0.5);
  auto diameter = morison->GetDiameter();
  EXPECT_FLOAT_EQ(0.5, diameter);
}

TEST_F(TestMorison, CompositionElementGeneralProperty) {
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);

  auto morison = std::make_shared<FrMorisonCompositeElement>(body.get());

  morison->SetAddedMass(m_addedMass);
  morison->SetDiameter(m_diameter);
  morison->SetDragCoeff(m_dragCoeff);
  morison->SetFrictionCoeff(m_frictionCoeff);

  morison->AddElement(m_pointA, m_pointB);
  auto force = make_morison_force("Morison", body, morison);
  system.Initialize();

  force->Update(0.);
  CheckForce(force.get());
}


TEST_F(TestMorison, CompositionElementGeneralPropertyNode) {
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);

  auto morison = std::make_shared<FrMorisonCompositeElement>(body.get());

  morison->SetAddedMass(m_addedMass);
  morison->SetDiameter(m_diameter);
  morison->SetDragCoeff(m_dragCoeff);
  morison->SetFrictionCoeff(m_frictionCoeff);

  auto nodeA = body->NewNode("nodeA");
  nodeA->SetPositionInBody(m_pointA, NWU);
  auto nodeB = body->NewNode("nodeB");
  nodeB->SetPositionInBody(m_pointB, NWU);

  morison->AddElement(nodeA, nodeB);
  auto force = make_morison_force("Morison", body, morison);
  system.Initialize();

  force->Update(0.);
  CheckForce(force.get());
}

