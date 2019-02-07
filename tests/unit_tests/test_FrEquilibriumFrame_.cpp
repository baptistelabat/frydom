//
// Created by camille on 23/11/18.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

using namespace frydom;

class TestFrEquilibriumFrame : public testing::Test {

protected:

    FrOffshoreSystem_ system;
    std::shared_ptr<FrBody_> body;
    std::shared_ptr<FrEquilibriumFrame_> m_eqFrame;

    Position m_PositionInWorld;
    Velocity m_VelocityInWorld;
    Velocity m_VelocityInFrame;

    FrUnitQuaternion_ m_quat;
    FrFrame_ m_frame;

    double m_angularVelocity = 0.01;

protected:
    /// Initialization of the environment
    void SetUp() override;

    /// Load reference results from GDF5 file
    void LoadData(std::string filename, std::string group);

    /// Vector reader
    template <class Vector>
    Vector ReadVector(FrHDF5Reader& reader, std::string field) const;

    void CheckVelocity();

public:
    void TestSetVelocityInWorld();
    void TestSetVelocityInFrame();
    void TestAngularVelocityAroundZ(double val);
    void TestGetVelocityInWorld();
    void TestGetVelocityInFrame();
    void TestGetAngularVelocity(double val);
    void TestInitSpeedFromBody();
    void TestInitPositionFromBody();
    void TestSetPositionToBodyPosition();
    void TestSetVelocityToBodyVelocity();
};

template <class Vector>
Vector TestFrEquilibriumFrame::ReadVector(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return Vector(value(0), value(1), value(2));
}

void TestFrEquilibriumFrame::SetUp() {
    LoadData("TNR_database.h5", "/equilibrium_frame/");

    body = std::make_shared<FrBody_>();
    body->SetPosition(m_PositionInWorld, NWU);
    body->SetRotation(m_quat);

    m_eqFrame = std::make_unique<FrEquilibriumFrame_>();
    m_eqFrame->SetPosition(m_PositionInWorld, NWU);
    m_eqFrame->SetRotation(m_quat);
    m_eqFrame->SetBody(body.get());

    m_eqFrame->Initialize();
    body->Initialize();
}

void TestFrEquilibriumFrame::CheckVelocity() {
    auto velocity  = m_eqFrame->GetVelocityInWorld(NWU);
    EXPECT_FLOAT_EQ(m_VelocityInWorld.GetVx(), velocity.GetVx());
    EXPECT_FLOAT_EQ(m_VelocityInWorld.GetVy(), velocity.GetVy());
    EXPECT_FLOAT_EQ(m_VelocityInWorld.GetVz(), velocity.GetVz());
}


void  TestFrEquilibriumFrame::LoadData(std::string filename, std::string group) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);

    m_PositionInWorld = ReadVector<Position>(reader, group + "PointInWorld");
    m_VelocityInWorld = ReadVector<Velocity>(reader, group + "VelocityInWorld");
    m_VelocityInFrame = ReadVector<Velocity>(reader, group + "VelocityInFrame");

    auto direction = ReadVector<Direction>(reader, group + "RotationDirection");
    direction.normalize();
    auto angle = reader.ReadDouble(group + "RotationAngle");
    m_quat = FrUnitQuaternion_(direction, angle, NWU);
    m_frame = FrFrame_(m_PositionInWorld, m_quat, NWU);
}

void TestFrEquilibriumFrame::TestSetVelocityInWorld() {
    m_eqFrame->SetVelocityInWorld(m_VelocityInWorld, NWU);
    CheckVelocity();
}

void TestFrEquilibriumFrame::TestSetVelocityInFrame() {
    m_eqFrame->SetVelocityInFrame(m_VelocityInFrame);
    CheckVelocity();
}

void TestFrEquilibriumFrame::TestAngularVelocityAroundZ(double val) {
    m_eqFrame->SetAngularVelocityAroundZ(val, NWU);
    EXPECT_FLOAT_EQ(val, m_eqFrame->GetAngularVelocityAroundZ(NWU));
}

void TestFrEquilibriumFrame::TestGetVelocityInWorld() {
    m_eqFrame->SetVelocityInWorld(m_VelocityInWorld, NWU);
    auto velocity = m_eqFrame->GetVelocityInWorld(NWU);
    EXPECT_FLOAT_EQ(m_VelocityInWorld.GetVx(), velocity.GetVx());
    EXPECT_FLOAT_EQ(m_VelocityInWorld.GetVy(), velocity.GetVy());
    EXPECT_FLOAT_EQ(m_VelocityInWorld.GetVz(), velocity.GetVz());
}

void TestFrEquilibriumFrame::TestGetVelocityInFrame() {
    m_eqFrame->SetVelocityInWorld(m_VelocityInWorld, NWU);
    auto velocity = m_eqFrame->GetVelocityInFrame();
    EXPECT_FLOAT_EQ(m_VelocityInFrame.GetVx(), velocity.GetVx());
    EXPECT_FLOAT_EQ(m_VelocityInFrame.GetVy(), velocity.GetVy());
    EXPECT_FLOAT_EQ(m_VelocityInFrame.GetVz(), velocity.GetVz());
}

void TestFrEquilibriumFrame::TestGetAngularVelocity(double val) {
    m_eqFrame->SetAngularVelocityAroundZ(val, NWU);
    auto angularVelocity = m_eqFrame->GetAngularVelocity(NWU);
    EXPECT_FLOAT_EQ(0., angularVelocity.GetWx());
    EXPECT_FLOAT_EQ(0., angularVelocity.GetWy());
    EXPECT_FLOAT_EQ(val, angularVelocity.GetWz());
}

void TestFrEquilibriumFrame::TestInitSpeedFromBody() {
    body->SetGeneralizedVelocityInWorld(m_VelocityInWorld, AngularVelocity(0., 0., m_angularVelocity), NWU);
    m_eqFrame->InitSpeedFromBody(true);
    m_eqFrame->Initialize();
    CheckVelocity();
    EXPECT_FLOAT_EQ(0., m_eqFrame->GetAngularVelocityAroundZ(NWU));
}

void TestFrEquilibriumFrame::TestInitPositionFromBody() {
    m_eqFrame->InitPositionFromBody(true);
    m_eqFrame->Initialize();

    auto position = m_eqFrame->GetPosition(NWU);
    EXPECT_FLOAT_EQ(m_PositionInWorld.GetX(), position.GetX());
    EXPECT_FLOAT_EQ(m_PositionInWorld.GetY(), position.GetY());
    EXPECT_FLOAT_EQ(m_PositionInWorld.GetZ(), position.GetZ());
}


void TestFrEquilibriumFrame::TestSetPositionToBodyPosition() {
    m_eqFrame->SetPositionToBodyPosition();

    auto position = m_eqFrame->GetPosition(NWU);
    EXPECT_FLOAT_EQ(m_PositionInWorld.GetX(), position.GetX());
    EXPECT_FLOAT_EQ(m_PositionInWorld.GetY(), position.GetY());
    EXPECT_FLOAT_EQ(m_PositionInWorld.GetZ(), position.GetZ());

    double q0, q1, q2, q3;
    double qr0, qr1, qr2, qr3;
    m_eqFrame->GetRotation().GetQuaternion().Get(q0, q1, q2, q3, NWU);
    m_quat.Get(qr0, qr1, qr2, qr3, NWU);
    EXPECT_FLOAT_EQ(q0, qr0);
    EXPECT_FLOAT_EQ(q1, qr1);
    EXPECT_FLOAT_EQ(q2, qr2);
    EXPECT_FLOAT_EQ(q3, qr3);
}

void TestFrEquilibriumFrame::TestSetVelocityToBodyVelocity() {
    body->SetGeneralizedVelocityInWorld(m_VelocityInWorld, AngularVelocity(0., 0., m_angularVelocity), NWU);

    m_eqFrame->SetVelocityToBodyVelocity();

    CheckVelocity();
    EXPECT_FLOAT_EQ(0., m_eqFrame->GetAngularVelocityAroundZ(NWU));
}

TEST_F(TestFrEquilibriumFrame, SetVelocityInWorld) {
    TestSetVelocityInWorld();
}

TEST_F(TestFrEquilibriumFrame, SetVelocityInFrame) {
    TestSetVelocityInFrame();
}

TEST_F(TestFrEquilibriumFrame, AngularVelocityAroundZ) {
    TestAngularVelocityAroundZ(0.01);
}

TEST_F(TestFrEquilibriumFrame, GetVelocityInWorld) {
    TestGetVelocityInWorld();
}

TEST_F(TestFrEquilibriumFrame, GetVelocityInFrame) {
    TestGetVelocityInFrame();
}

TEST_F(TestFrEquilibriumFrame, GetAngularVelocity) {
    TestGetAngularVelocity(0.01);
}

TEST_F(TestFrEquilibriumFrame, InitPositionFromBody) {
    TestInitPositionFromBody();
}

TEST_F(TestFrEquilibriumFrame, InitSpeedFromBody) {
    TestInitSpeedFromBody();
}

TEST_F(TestFrEquilibriumFrame, SetPositionToBodyPosition) {
    TestSetPositionToBodyPosition();
}

TEST_F(TestFrEquilibriumFrame, SetVelocityToBodyVelocity) {
    TestSetVelocityToBodyVelocity();
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}