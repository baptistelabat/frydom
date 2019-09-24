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

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

using namespace frydom;

class TestFrEquilibriumFrame : public testing::Test {

protected:
    
    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system;
    std::shared_ptr<FrBody> body;
    std::shared_ptr<FrEquilibriumFrame> m_eqFrame;

    Position m_PositionInWorld;
    Velocity m_VelocityInWorld;
    Velocity m_VelocityInFrame;

    FrUnitQuaternion m_quat;
    FrFrame m_frame;

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
    void TestDefaultInitialization();
    void TestInitSpeedFromBody();
//    void TestInitPositionFromBody();
//    void TestSetPositionToBodyPosition();
    void TestSetVelocityToBodyVelocity();
};

template <class Vector>
Vector TestFrEquilibriumFrame::ReadVector(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return Vector(value(0), value(1), value(2));
}

void TestFrEquilibriumFrame::SetUp() {
    system.GetPathManager()->SetResourcesPath(std::string(RESOURCES_PATH));
    LoadData(system.GetDataPath("TNR_database.h5"), "/equilibrium_frame/");

    body = system.NewBody();
    body->SetPosition(m_PositionInWorld, fc);
    body->SetRotation(m_quat);

    m_eqFrame = std::make_unique<FrEquilibriumFrame>(body.get());
    m_eqFrame->SetPositionInWorld(m_PositionInWorld, fc);
    m_eqFrame->SetRotation(FrRotation(m_quat));

    system.Add(m_eqFrame);

    system.Initialize();
}

void TestFrEquilibriumFrame::CheckVelocity() {
    auto velocity  = m_eqFrame->GetVelocityInWorld(fc);
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
    m_quat = FrUnitQuaternion(direction, angle, fc);
    m_frame = FrFrame(m_PositionInWorld, m_quat, fc);
}

void TestFrEquilibriumFrame::TestSetVelocityInWorld() {
    m_eqFrame->SetVelocityInWorld(m_VelocityInWorld, fc);
    CheckVelocity();
}

void TestFrEquilibriumFrame::TestSetVelocityInFrame() {
    m_eqFrame->SetVelocityInFrame(m_VelocityInFrame, fc);
    CheckVelocity();
}

void TestFrEquilibriumFrame::TestAngularVelocityAroundZ(double val) {
    m_eqFrame->SetAngularVelocityAroundZ(val, fc);
    EXPECT_FLOAT_EQ(val, m_eqFrame->GetAngularVelocityAroundZ(fc));
}

void TestFrEquilibriumFrame::TestGetVelocityInWorld() {
    m_eqFrame->SetVelocityInWorld(m_VelocityInWorld, fc);
    auto velocity = m_eqFrame->GetVelocityInWorld(fc);
    EXPECT_FLOAT_EQ(m_VelocityInWorld.GetVx(), velocity.GetVx());
    EXPECT_FLOAT_EQ(m_VelocityInWorld.GetVy(), velocity.GetVy());
    EXPECT_FLOAT_EQ(m_VelocityInWorld.GetVz(), velocity.GetVz());
}

void TestFrEquilibriumFrame::TestGetVelocityInFrame() {
    m_eqFrame->SetVelocityInWorld(m_VelocityInWorld, fc);
    auto velocity = m_eqFrame->GetVelocityInFrame(fc);
    EXPECT_FLOAT_EQ(m_VelocityInFrame.GetVx(), velocity.GetVx());
    EXPECT_FLOAT_EQ(m_VelocityInFrame.GetVy(), velocity.GetVy());
    EXPECT_FLOAT_EQ(m_VelocityInFrame.GetVz(), velocity.GetVz());
}

void TestFrEquilibriumFrame::TestGetAngularVelocity(double val) {
    m_eqFrame->SetAngularVelocityAroundZ(val, fc);
    auto angularVelocity = m_eqFrame->GetAngularVelocity(fc);
    EXPECT_FLOAT_EQ(0., angularVelocity.GetWx());
    EXPECT_FLOAT_EQ(0., angularVelocity.GetWy());
    EXPECT_FLOAT_EQ(val, angularVelocity.GetWz());
}

void TestFrEquilibriumFrame::TestDefaultInitialization() {
//    m_eqFrame->Initialize();
    auto velocity  = m_eqFrame->GetVelocityInWorld(fc);
    EXPECT_FLOAT_EQ(0., velocity.GetVx());
    EXPECT_FLOAT_EQ(0., velocity.GetVy());
    EXPECT_FLOAT_EQ(0., velocity.GetVz());
    EXPECT_FLOAT_EQ(0., m_eqFrame->GetAngularVelocityAroundZ(fc));

    auto position = m_eqFrame->GetPositionInWorld(fc);
    EXPECT_FLOAT_EQ(m_PositionInWorld.GetX(), position.GetX());
    EXPECT_FLOAT_EQ(m_PositionInWorld.GetY(), position.GetY());
    EXPECT_FLOAT_EQ(m_PositionInWorld.GetZ(), position.GetZ());

    double q0, q1, q2, q3;
    double qr0, qr1, qr2, qr3;
    m_eqFrame->GetRotation().GetQuaternion().Get(q0, q1, q2, q3, fc);
    m_quat.Get(qr0, qr1, qr2, qr3, fc);
    EXPECT_FLOAT_EQ(q0, qr0);
    EXPECT_FLOAT_EQ(q1, qr1);
    EXPECT_FLOAT_EQ(q2, qr2);
    EXPECT_FLOAT_EQ(q3, qr3);

}

void TestFrEquilibriumFrame::TestInitSpeedFromBody() {
    body->SetGeneralizedVelocityInWorld(m_VelocityInWorld, AngularVelocity(0., 0., m_angularVelocity), fc);
    m_eqFrame->InitSpeedFromBody(true);
    m_eqFrame->Initialize();
    CheckVelocity();
    EXPECT_FLOAT_EQ(0., m_eqFrame->GetAngularVelocityAroundZ(fc));
}

//void TestFrEquilibriumFrame::TestInitPositionFromBody() {
////    m_eqFrame->InitPositionFromBody(true);
//    m_eqFrame->Initialize();
//
//    auto position = m_eqFrame->GetPositionInWorld(fc);
//    EXPECT_FLOAT_EQ(m_PositionInWorld.GetX(), position.GetX());
//    EXPECT_FLOAT_EQ(m_PositionInWorld.GetY(), position.GetY());
//    EXPECT_FLOAT_EQ(m_PositionInWorld.GetZ(), position.GetZ());
//}
//
//
//void TestFrEquilibriumFrame::TestSetPositionToBodyPosition() {
//    m_eqFrame->SetPositionToBodyCOGPosition();
//
//    auto position = m_eqFrame->GetPosition(fc);
//    EXPECT_FLOAT_EQ(m_PositionInWorld.GetX(), position.GetX());
//    EXPECT_FLOAT_EQ(m_PositionInWorld.GetY(), position.GetY());
//    EXPECT_FLOAT_EQ(m_PositionInWorld.GetZ(), position.GetZ());
//
//    double q0, q1, q2, q3;
//    double qr0, qr1, qr2, qr3;
//    m_eqFrame->GetRotation().GetQuaternion().Get(q0, q1, q2, q3, fc);
//    m_quat.Get(qr0, qr1, qr2, qr3, fc);
//    EXPECT_FLOAT_EQ(q0, qr0);
//    EXPECT_FLOAT_EQ(q1, qr1);
//    EXPECT_FLOAT_EQ(q2, qr2);
//    EXPECT_FLOAT_EQ(q3, qr3);
//}

void TestFrEquilibriumFrame::TestSetVelocityToBodyVelocity() {
    body->SetGeneralizedVelocityInWorld(m_VelocityInWorld, AngularVelocity(0., 0., m_angularVelocity), fc);
    m_eqFrame->SetVelocityToBodyCOGVelocity();
    CheckVelocity();
    EXPECT_FLOAT_EQ(0., m_eqFrame->GetAngularVelocityAroundZ(fc));
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

TEST_F(TestFrEquilibriumFrame, DefaultInitialization) {
    TestDefaultInitialization();
}

//TEST_F(TestFrEquilibriumFrame, InitPositionFromBody) {
//    TestInitPositionFromBody();
//}

TEST_F(TestFrEquilibriumFrame, InitSpeedFromBody) {
    TestInitSpeedFromBody();
}

//TEST_F(TestFrEquilibriumFrame, SetPositionToBodyPosition) {
//    TestSetPositionToBodyPosition();
//}

TEST_F(TestFrEquilibriumFrame, SetVelocityToBodyVelocity) {
    TestSetVelocityToBodyVelocity();
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
