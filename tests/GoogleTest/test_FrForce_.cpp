//
// Created by camille on 08/11/18.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;

//
// CLASS OF THE TEST
//

class TestFrForce_ : public FrForce_ {

private:
    Position m_PointCOGInWorld;                 ///< Position of the COG in world
    Position m_PointCOGInBody;                  ///< Position of the COG in body

    Position m_PointREFInWorld;                 ///< Position of the body in world

    FrFrame_ m_frameREF;                        ///< Body frame
    FrQuaternion_ m_quatREF;                    ///< Rotation of the body in world

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

public:
    /// Test
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

    /// Accessor
    Position GetPointREFInWorld() const { return m_PointREFInWorld; }
    Position GetPointCOGInBody()  const { return m_PointCOGInBody; }
    FrQuaternion_ GetQuatREF()    const { return m_quatREF; }

    void CreateDataset();

private:
    /// Override pure virtual methods
    void Update(double time) override {};
    void Initialize() override {};
    void StepFinalize() override {};
};

void TestFrForce_::CreateDataset() {

    /// Positions
    m_PointREFInWorld   = Position(10., 20., -5.);
    m_PointCOGInBody    = Position(-1., 2., 3.);
    m_PointInBody       = Position(5., 10., 3.);
    m_PointCOGInWorld   = Position(9.02460532, 21.06709459, -1.54892562);
    m_PointInWorld = Position(12.40247008, 30.43053193, -0.59180977);

    /// Frame of the body
    Direction vect = Direction(0.5, 0.4, 0.8);
    vect.normalize();
    double angle = M_PI/8.;
    m_quatREF =  FrQuaternion_(vect, angle, NWU);
    m_frameREF = FrFrame_(m_PointREFInWorld, m_quatREF, NWU);  // TODO : a voir si c'est utile de définir la convention ici

    /// Force and Torque
    m_forceInWorldAtPoint  = Force(1761428.06341033,5290133.96192031,2629040.47940839);
    m_forceInBodyAtPoint   = Force(3000000.00000000,5000000.00000000,2000000.00000000);
    m_forceInWorldAtCOG    = Force(1761428.06341033,5290133.96192031,2629040.47940839);
    m_forceInBodyAtCOG     = Force(3000000.00000000,5000000.00000000,2000000.00000000);
    m_torqueInWorldAtPoint = Torque(16157552865.99235725,-14374562644.35187531,87363810780.93070984);
    m_torqueInBodyAtPoint  = Torque(200000000.00000000,300000000.00000000,90000000000.00000000);
    m_torqueInWorldAtCOG   = Torque(16177106450.72056389,-14381757296.81767845,87365187116.70848083);
    m_torqueInBodyAtCOG    = Torque(216000000.00000000,288000000.00000000,90006000000.00000000);

    return;
}

void TestFrForce_::TestMaxForceLimit(double fmax) {
    this->SetMaxForceLimit(fmax);
    EXPECT_FLOAT_EQ(this->GetMaxForceLimit(), fmax);
    return;
}

void TestFrForce_::TestMaxTorqueLimit(double fmax) {
    this->SetMaxTorqueLimit(fmax);
    EXPECT_FLOAT_EQ(this->GetMaxTorqueLimit(), fmax);
    return;
}

void TestFrForce_::TestSetLimit() {
    this->SetLimit(true);
    EXPECT_TRUE(this->GetLimit());
    return;
}

void TestFrForce_::TestForceTorqueInWorldAtPointInBody() {

    this->SetForceTorqueInWorldAtPointInBody(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInBody, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    return;
}


void TestFrForce_::TestForceTorqueInBodyAtPointInBody() {

    this->SetForceTorqueInBodyAtPointInBody(m_forceInBodyAtPoint, m_torqueInBodyAtPoint, m_PointInBody, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    return;
}

void TestFrForce_::TestForceTorqueInWorldAtPointInWorld() {

    this->SetForceTorqueInWorldAtPointInWorld(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInWorld, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    return;
}


void TestFrForce_::TestForceTorqueInBodyAtPointInWorld() {

    this->SetForceTorqueInBodyAtPointInWorld(m_forceInBodyAtPoint, m_torqueInBodyAtPoint, m_PointInWorld, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    return;
}

void TestFrForce_::TestForceTorqueInWorldAtCOG() {

    this->SetForceTorqueInWorldAtCOG(m_forceInWorldAtCOG, m_torqueInWorldAtCOG, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    return;
}

void TestFrForce_::TestForceTorqueInBodyAtCOG() {

    this->SetForceTorqueInBodyAtCOG(m_forceInBodyAtCOG, m_torqueInBodyAtCOG, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    return;
}


void TestFrForce_::TestTorqueInBodyAtCOG() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);

    m_body->Update();

    auto torque = this->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    return;
}


void TestFrForce_::TestTorqueInWorldAtCOG() {

    this->SetTorqueInWorldAtCOG(m_torqueInWorldAtCOG, NWU);

    m_body->Update();

    auto torque = this->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    return;
}

void TestFrForce_::TestForceInBodyAtCOG() {

    this->SetForceInBody(m_forceInBodyAtCOG, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    return;
}

void TestFrForce_::TestForceInWorldAtCOG() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    return;

}

void TestFrForce_::TestForceInWorldAtPointInBody() {

    this->SetForceInWorldAtPointInBody(m_forceInWorldAtPoint, m_PointInBody, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());

    return;

}

void TestFrForce_::TestForceInWorldAtPointInWorld() {

    this->SetForceInWorldAtPointInWorld(m_forceInWorldAtPoint, m_PointInWorld, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());

    return;
}

void TestFrForce_::TestForceInBodyAtPointInBody() {

    this->SetForceInBodyAtPointInBody(m_forceInBodyAtPoint, m_PointInBody, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());

    return;
}

void TestFrForce_::TestForceInBodyAtPointInWorld() {

    this->SetForceInBodyAtPointInWorld(m_forceInBodyAtPoint, m_PointInWorld, NWU);

    m_body->Update();

    auto force = this->GetForceInWorld(NWU);
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());

    return;
}

void TestFrForce_::TestGetForceInWorldReference() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInWorld(force, NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    return;

}

void TestFrForce_::TestGetForceInWorldComponent() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInWorld(force.GetFx(), force.GetFy(), force.GetFz(), NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    return;
}

void TestFrForce_::TestGetForceInBody() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    auto force = GetForceInBody(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

    return;
}

void TestFrForce_::TestGetForceInBodyReference() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInBody(force, NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

    return;
}


void TestFrForce_::TestGetForceInBodyComponent() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInBody(force.GetFx(), force.GetFy(), force.GetFz(), NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

    return;
}

std::shared_ptr<FrBody_> NewBody(std::shared_ptr<TestFrForce_> test) {

    auto body = std::make_shared<FrBody_>();
    body->SetPosition(test->GetPointREFInWorld(), NWU);
    body->SetCOG(test->GetPointCOGInBody(), NWU);
    body->SetRotation(test->GetQuatREF());
    body->AddExternalForce(test);
    body->Update();
    return body;
}

//
// GTEST
//

TEST(FrForce_test, MaxForceLimit) {
    TestFrForce_ test;
    test.TestMaxForceLimit(5.e6);
}

TEST(FrForce_test, MaxTorqueLimit) {
    TestFrForce_ test;
    test.TestMaxTorqueLimit(9.e9);
}

TEST(FrForce_test, SetLimit) {
    TestFrForce_ test;
    test.TestSetLimit();
}

TEST(FrForce_test, ForceTorqueInWorldAtPointInBody) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceTorqueInWorldAtPointInBody();
}

TEST(FrForce_test, ForceTorqueInBodyAtPointInBody) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceTorqueInBodyAtPointInBody();
}

TEST(FrForce_test, ForceTorqueInWorldAtPointInWorld) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceTorqueInWorldAtPointInWorld();
}

TEST(FrForce_test, ForceTorqueInBodyAtPointInWorld) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceTorqueInBodyAtPointInWorld();
}


TEST(FrForce_test, ForceTorqueInWorldAtCOG) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceTorqueInWorldAtCOG();
}

TEST(FrForce_test, ForceTorqueInBodyAtCOG) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceTorqueInBodyAtCOG();
}

TEST(FrForce_test, TorqueInBodyAtCOG) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestTorqueInBodyAtCOG();
}

TEST(FrForce_test, TorqueInWorldAtCOG) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestTorqueInWorldAtCOG();
}

TEST(FrForce_test, ForceInBodyAtCOG) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceInBodyAtCOG();
}

TEST(FrForce_test, ForceInWorldAtCOG) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceInWorldAtCOG();
}

TEST(FrForce_test, ForceInWorldAtPointInBody) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceInWorldAtPointInBody();
}

TEST(FrForce_test, ForceInWorldAtPointInWorld) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceInWorldAtPointInWorld();
}

TEST(FrForce_test, ForceInBodyAtPointInBody) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceInBodyAtPointInBody();
}

TEST(FrForce_test, ForceInBodyAtPointInWorld) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestForceInBodyAtPointInWorld();
}

TEST(FrForce_test, GetForceInWorldReference) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestGetForceInWorldReference();
}

TEST(FrForce_test, GetForceInWorldComponent) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestGetForceInWorldComponent();
}

TEST(FrForce_test, GetForceInBody) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestGetForceInBody();
}

TEST(FrForce_test, GetForceInBodyReference) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestGetForceInBodyReference();
}

TEST(FrForce_test, GetForceInBodyComponent) {
    FrOffshoreSystem_ system;
    auto test = std::make_shared<TestFrForce_>();
    test->CreateDataset();
    auto body = NewBody(test);
    system.AddBody(body);
    test->TestGetForceInBodyComponent();
}