//
// Created by camille on 29/11/18.
//

#include <memory>
#include "frydom/frydom.h"
#include "gtest/gtest.h"
#include "frydom/core/FrVector.h"

using namespace frydom;

// ---------------------------------------------------------------
//
// MORISON SINGLE ELEMENT
//
// ---------------------------------------------------------------

class TestMorisonSingleElement : public FrMorisonSingleElement_ {

public:
    TestMorisonSingleElement(FrBody_* body) : FrMorisonSingleElement_(body) { }

    TestMorisonSingleElement(FrBody_* body, Position posA, Position posB, double diameter,
                             MorisonCoeff ca, MorisonCoeff cd, double cf,
                             Direction perpendicular = Direction(0., 0. ,1.));

    TestMorisonSingleElement(std::shared_ptr<FrNode_> nodeA, std::shared_ptr<FrNode_> nodeB,
                             double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                             Direction perpendicular = Direction(0., 0., 1.));

    void TestAddedMass(double ca);
    void TestAddedMass(double caX, double caY);
    void TestDragCoeff(double cd);
    void TestDragCoeff(double cdX, double cdY);
    void TestFriction(double cf);
    void CheckDiameter(double d);
    void CheckNodesPointer(std::shared_ptr<FrNode_> nodeA, std::shared_ptr<FrNode_> nodeB);
    void CheckNodesPointer(FrBody_* body, Position posA, Position posB);
    void CheckLength(double l);
    void CheckAddedMass(double caX, double caY);
    void CheckDragCoeff(double cdX, double cdY);
    void CheckFriction(double cf);
};

TestMorisonSingleElement::TestMorisonSingleElement(FrBody_ *body, Position posA, Position posB, double diameter,
                                                   MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                   Direction perpendicular)
    : FrMorisonSingleElement_(body, posA, posB, diameter, ca, cd, cf, perpendicular)   {

    CheckDiameter(diameter);
    CheckAddedMass(ca.x, ca.y);
    CheckDragCoeff(cd.x, cd.y);
    CheckFriction(cf);
    CheckLength((posB-posA).norm());
}

TestMorisonSingleElement::TestMorisonSingleElement(std::shared_ptr<FrNode_> nodeA, std::shared_ptr<FrNode_> nodeB,
                                                   double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                   Direction perpendicular)
        : FrMorisonSingleElement_(nodeA, nodeB, diameter, ca, cd, cf, perpendicular)   {

    CheckNodesPointer(nodeA, nodeB);
    CheckDiameter(diameter);
    CheckAddedMass(ca.x, ca.y);
    CheckDragCoeff(cd.x, cd.y);
    CheckFriction(cf);
    CheckLength((nodeB->GetPositionInWorld(NWU)-nodeA->GetPositionInWorld(NWU)).norm());
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

void TestMorisonSingleElement::CheckNodesPointer(std::shared_ptr<FrNode_> nodeA, std::shared_ptr<FrNode_> nodeB) {
    EXPECT_EQ( m_nodeA, nodeA);
    EXPECT_EQ( m_nodeB, nodeB);
}

void TestMorisonSingleElement::CheckNodesPointer(FrBody_* body, Position posA, Position posB) {
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

// -----------------------------------------------------------------
//
// MORISON TEST
//
// -----------------------------------------------------------------

class TestMorison : public ::testing::Test {

public:
    FrOffshoreSystem_ system;
    std::shared_ptr<FrBody_> body;

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
    void SetUp() override;
    void LoadData(std::string filename);
    template <class Vector>
    Vector ReadVector(FrHDF5Reader& reader, std::string field) const;
    MorisonCoeff ReadMorisonCoeff(FrHDF5Reader& reader, std::string field) const;
    void CheckForce(FrForce_* force) const;
};

template <class Vector>
Vector TestMorison::ReadVector(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return Vector(value(0), value(1), value(2));
}

MorisonCoeff TestMorison::ReadMorisonCoeff(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return MorisonCoeff(value(0), value(1));
}

void TestMorison::SetUp() {
    body = std::make_shared<FrBody_>();
    system.AddBody(body);
}

void TestMorison::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "/morison/";

    auto waveHeight = reader.ReadDouble(group + "WaveHeight");
    auto wavePeriod = reader.ReadDouble(group + "WavePeriod");

    auto waveField = system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWaveHeight(waveHeight);
    waveField->SetWavePeriod(wavePeriod, S);

    auto pointRef = ReadVector<Position>(reader, group + "PointRef");
    body->SetPosition(pointRef, NWU);

    auto posCOG = ReadVector<Position>(reader, group + "COG");
    body->SetCOG(posCOG, NWU);

    auto direction = ReadVector<Direction>(reader, group + "RotationDirection");
    direction.normalize();
    auto angle = reader.ReadDouble(group + "RotationAngle");
    body->SetRotation(FrUnitQuaternion_(direction, angle, NWU));

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

void TestMorison::CheckForce(FrForce_* force) const {

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
    auto nodeA = std::make_shared<FrNode_>(body.get(), posA);
    auto nodeB = std::make_shared<FrNode_>(body.get(), posB);

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

    auto nodeA = std::make_shared<FrNode_>(body.get(), posA);
    auto nodeB = std::make_shared<FrNode_>(body.get(), posB);
    auto morison2 = std::make_shared<TestMorisonSingleElement>(nodeA, nodeB, 0.5, 0.1, 0.2, 0.01);

}

TEST_F(TestMorison, UpdateForce) {

    LoadData("TNR_database.h5");

    auto morison = std::make_shared<TestMorisonSingleElement>(body.get(), m_pointA, m_pointB, m_diameter, m_addedMass, m_dragCoeff,
                                                              m_frictionCoeff);

    auto force = std::make_shared<FrMorisonForce_>(morison);

    system.AddPhysicsItem(morison);
    body->AddExternalForce(force);
    system.Initialize();

    force->Update(0.);
    CheckForce(force.get());
}




