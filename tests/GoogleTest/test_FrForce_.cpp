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
    void TestForceNorm();
    void TestTorqueNorm();

    /// Accessor
    Position GetPointREFInWorld() const { return m_PointREFInWorld; }
    Position GetPointCOGInBody()  const { return m_PointCOGInBody; }
    FrQuaternion_ GetQuatREF()    const { return m_quatREF; }

    /// Methods
    void CreateDataset();
    void CheckForceInWorldAtCOG();
    void CheckTorqueInBodyAtCOG();
    void CheckForceTorqueAtCOG();

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
}

void TestFrForce_::CheckForceInWorldAtCOG() {

    m_body->Update();
    auto force = this->GetForceInWorld(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());
}

void TestFrForce_::CheckTorqueInBodyAtCOG() {

    m_body->Update();
    auto torque = this->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());
}

void TestFrForce_::CheckForceTorqueAtCOG() {
    this->CheckForceInWorldAtCOG();
    this->CheckTorqueInBodyAtCOG();
}



void TestFrForce_::TestMaxForceLimit(double fmax) {
    this->SetMaxForceLimit(fmax);
    EXPECT_FLOAT_EQ(this->GetMaxForceLimit(), fmax);
}

void TestFrForce_::TestMaxTorqueLimit(double fmax) {
    this->SetMaxTorqueLimit(fmax);
    EXPECT_FLOAT_EQ(this->GetMaxTorqueLimit(), fmax);
}

void TestFrForce_::TestSetLimit() {
    this->SetLimit(true);
    EXPECT_TRUE(this->GetLimit());
}

void TestFrForce_::TestForceTorqueInWorldAtPointInBody() {
    this->SetForceTorqueInWorldAtPointInBody(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInBody, NWU);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestForceTorqueInBodyAtPointInBody() {
    this->SetForceTorqueInBodyAtPointInBody(m_forceInBodyAtPoint, m_torqueInBodyAtPoint, m_PointInBody, NWU);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestForceTorqueInWorldAtPointInWorld() {
    this->SetForceTorqueInWorldAtPointInWorld(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInWorld, NWU);
    CheckForceTorqueAtCOG();
}


void TestFrForce_::TestForceTorqueInBodyAtPointInWorld() {
    this->SetForceTorqueInBodyAtPointInWorld(m_forceInBodyAtPoint, m_torqueInBodyAtPoint, m_PointInWorld, NWU);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestForceTorqueInWorldAtCOG() {
    this->SetForceTorqueInWorldAtCOG(m_forceInWorldAtCOG, m_torqueInWorldAtCOG, NWU);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestForceTorqueInBodyAtCOG() {
    this->SetForceTorqueInBodyAtCOG(m_forceInBodyAtCOG, m_torqueInBodyAtCOG, NWU);
    CheckForceTorqueAtCOG();
}


void TestFrForce_::TestTorqueInBodyAtCOG() {
    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    CheckTorqueInBodyAtCOG();
}


void TestFrForce_::TestTorqueInWorldAtCOG() {
    this->SetTorqueInWorldAtCOG(m_torqueInWorldAtCOG, NWU);
    CheckTorqueInBodyAtCOG();
}

void TestFrForce_::TestForceInBodyAtCOG() {
    this->SetForceInBody(m_forceInBodyAtCOG, NWU);
    CheckForceInWorldAtCOG();
}

void TestFrForce_::TestForceInWorldAtCOG() {
    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);
    CheckForceInWorldAtCOG();
}

void TestFrForce_::TestForceInWorldAtPointInBody() {

    this->SetForceInWorldAtPointInBody(m_forceInWorldAtPoint, m_PointInBody, NWU);

    CheckForceInWorldAtCOG();

    m_body->Update();
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

    EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());
}

void TestFrForce_::TestForceInWorldAtPointInWorld() {

    this->SetForceInWorldAtPointInWorld(m_forceInWorldAtPoint, m_PointInWorld, NWU);

    CheckForceInWorldAtCOG();

    m_body->Update();
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

    EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());
}

void TestFrForce_::TestForceInBodyAtPointInBody() {

    this->SetForceInBodyAtPointInBody(m_forceInBodyAtPoint, m_PointInBody, NWU);

    CheckForceInWorldAtCOG();

    m_body->Update();
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

    EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());
}

void TestFrForce_::TestForceInBodyAtPointInWorld() {

    this->SetForceInBodyAtPointInWorld(m_forceInBodyAtPoint, m_PointInWorld, NWU);

    CheckForceInWorldAtCOG();

    m_body->Update();
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

    EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());
}

void TestFrForce_::TestGetForceInWorldReference() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInWorld(force, NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());
}

void TestFrForce_::TestGetForceInWorldComponent() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInWorld(force.GetFx(), force.GetFy(), force.GetFz(), NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());
}

void TestFrForce_::TestGetForceInBody() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    auto force = GetForceInBody(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());
}

void TestFrForce_::TestGetForceInBodyReference() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInBody(force, NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());
}


void TestFrForce_::TestGetForceInBodyComponent() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);
    m_body->Update();

    Force force;
    GetForceInBody(force.GetFx(), force.GetFy(), force.GetFz(), NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

}


void TestFrForce_::TestGetTorqueInWorld() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    auto torque = GetTorqueInWorldAtCOG(NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG.GetMz());
}

void TestFrForce_::TestGetTorqueInWorldReference() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    Torque torque;
    GetTorqueInWorldAtCOG(torque, NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG.GetMz());
}

void TestFrForce_::TestGetTorqueInWorldComponent() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    Torque torque;
    GetTorqueInWorldAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG.GetMz());
}

void TestFrForce_::TestGetTorqueInBodyReference() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    Torque torque;
    GetTorqueInBodyAtCOG(torque, NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());
}

void TestFrForce_::TestGetTorqueInBodyComponent() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    Torque torque;
    GetTorqueInBodyAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());
}


void TestFrForce_::TestForceNorm() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);
    m_body->Update();

    double normREF =  std::sqrt(m_forceInWorldAtCOG.GetFx() * m_forceInWorldAtCOG.GetFx()
                    + m_forceInWorldAtCOG.GetFy() * m_forceInWorldAtCOG.GetFy()
                    + m_forceInWorldAtCOG.GetFz() * m_forceInWorldAtCOG.GetFz());

    auto normTEST = GetForceNorm();

    EXPECT_FLOAT_EQ(normTEST, normREF);
}


void TestFrForce_::TestTorqueNorm() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    double normREF =  std::sqrt(m_torqueInBodyAtCOG.GetMx() * m_torqueInBodyAtCOG.GetMx()
                    + m_torqueInBodyAtCOG.GetMy() * m_torqueInBodyAtCOG.GetMy()
                    + m_torqueInBodyAtCOG.GetMz() * m_torqueInBodyAtCOG.GetMz());

    auto normTEST = GetTorqueNormAtCOG();

    EXPECT_FLOAT_EQ(normTEST, normREF);
}


//
// TESTING CLASS FUNCTION
//

class TestBase : public ::testing::Test {

protected:

    /// Environment objects
    FrOffshoreSystem_ system;
    std::shared_ptr<FrBody_> body;
    std::shared_ptr<TestFrForce_> force;

    /// Initialize environment
    void SetUp() override;

    /// Create a new body object with position and orientation of the test
    std::shared_ptr<FrBody_> NewBody(std::shared_ptr<TestFrForce_> test);

};

void TestBase::SetUp() {
    force = std::make_shared<TestFrForce_>();
    force->CreateDataset();
    body = NewBody(force);
    system.AddBody(body);
}

std::shared_ptr<FrBody_> TestBase::NewBody(std::shared_ptr<TestFrForce_> test) {

    auto body = std::make_shared<FrBody_>();
    body->SetPosition(test->GetPointREFInWorld(), NWU);
    body->SetCOG(test->GetPointCOGInBody(), NWU);
    body->SetRotation(test->GetQuatREF());
    body->AddExternalForce(test);
    body->Update();
    return body;
}


//
// LIST OF GTEST
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
    force->TestGetTorqueInWorldReference();
}

TEST_F(TestBase,GetTorqueInBodyComponent) {
    force->TestGetTorqueInBodyComponent();
}

TEST_F(TestBase, ForceNorm) {
    force->TestForceNorm();
}

TEST_F(TestBase, TorqueNorm) {
    force->TestTorqueNorm();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
