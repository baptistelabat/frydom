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
    void TestMaxForceLimit(double fmax);
    void TestMaxTorqueLimit(double fmax);
    void TestSetLimit();
    void TestAbsForceOnLocalPoint();
private:
    /// Override pure virtual methods
    void Update(double time) override {};
    void Initialize() override {};
    void StepFinalize() override {};
    void CreateDataset();
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

void TestFrForce_::TestAbsForceOnLocalPoint() {

    this->CreateDataset();

    m_body->SetAbsPosition(m_PointREFInWorld, NWU);
    m_body->SetCOGLocalPosition(m_PointCOGInBody, true, NWU);
    m_body->SetAbsRotation(m_quatREF);

    m_body->Update();

    this->SetAbsForceTorqueAtLocalPoint(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInBody, NWU);

    m_body->Update();

    auto force = this->GetAbsForce(NWU);
    auto torque = this->GetLocalTorqueAtCOG(NWU);

    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    return;
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

TEST(FrForce_test, AbsForceOnLocalPoint) {
    FrOffshoreSystem_ system;
    auto body = std::make_shared<FrBody_>();
    system.AddBody(body);
    auto test = std::make_shared<TestFrForce_>();
    body->AddExternalForce(test);
    test->TestAbsForceOnLocalPoint();
}