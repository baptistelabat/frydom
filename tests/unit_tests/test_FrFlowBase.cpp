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

#include "frydom/environment/flow/FrFlowBase.h"

using namespace frydom;

// --------------------------------------------------------------------
//
// TEST OF THE FLOW BASE
//
// --------------------------------------------------------------------

class TestFrFlowBase : public ::testing::Test {

protected:
    FrOffshoreSystem system;               ///< system environment
    std::shared_ptr<FrFlowBase> flow;       ///< flow instanciation for the test
    Velocity m_VelocityInWorld;             ///< Uniform velocity of the flow in world
    Velocity m_RelativeVelocityInFrame;     ///< Velocity of the flow relative to the frame express into the frame
    Position m_PointInWorld;                ///< Position of the frame in world (NWU)
    Velocity m_FrameVelocityInWorld;        ///< Velocity of the frame in world (NWU)
    FrUnitQuaternion_ m_quat;                   ///< Orientation of the frame (quaternion) / world
    FrFrame m_frame;                       ///< Local frame

protected:
    /// Initialization of the environment
    void SetUp() override;

    /// Load reference results from HDF5 file
    void LoadData(std::string filename);

    /// Vector reader
    template <class Vector>
    Vector ReadVector(FrHDF5Reader& reader, std::string field) const;

public:
    void TestGetFluxVelocityInWorld();
    void TestGetFluxVelocityInFrame();

    // NewField()
    // GetField()

};

template <class Vector>
Vector TestFrFlowBase::ReadVector(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return Vector(value(0), value(1), value(2));
}

void TestFrFlowBase::SetUp() {

    LoadData("TNR_database.h5");
    system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
    flow = std::make_shared<FrCurrent>(system.GetEnvironment()->GetOcean());
    flow->MakeFieldUniform();
    flow->GetFieldUniform()->Set(m_VelocityInWorld, NWU, GOTO);
}

void TestFrFlowBase::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "/flow_base/uniform/";

    m_PointInWorld = ReadVector<Position>(reader, group + "PointInWorld");
    auto direction = ReadVector<Direction>(reader, group + "RotationDirection");
    direction.normalize();
    auto angle = reader.ReadDouble(group + "RotationAngle");
    m_quat = FrUnitQuaternion_(direction, angle, NWU);
    m_frame = FrFrame(m_PointInWorld, m_quat, NWU);

    m_FrameVelocityInWorld = ReadVector<Velocity>(reader, group + "FrameVelocityInWorld");
    m_VelocityInWorld = ReadVector<Velocity>(reader, group + "VelocityInWorld");
    m_RelativeVelocityInFrame = ReadVector<Velocity>(reader, group + "RelativeVelocityInFrame");

}

void TestFrFlowBase::TestGetFluxVelocityInWorld() {

    auto velocity = flow->GetFluxVelocityInWorld(m_PointInWorld, NWU);
    EXPECT_FLOAT_EQ(velocity.GetVx(), m_VelocityInWorld.GetVx());
    EXPECT_FLOAT_EQ(velocity.GetVy(), m_VelocityInWorld.GetVy());
    EXPECT_FLOAT_EQ(velocity.GetVz(), m_VelocityInWorld.GetVz());

    velocity = flow->GetFluxVelocityInWorld(m_PointInWorld, NED);
    EXPECT_FLOAT_EQ(velocity.GetVx(), m_VelocityInWorld.GetVx());
    EXPECT_FLOAT_EQ(velocity.GetVy(), -m_VelocityInWorld.GetVy());
    EXPECT_FLOAT_EQ(velocity.GetVz(), -m_VelocityInWorld.GetVz());

}

void TestFrFlowBase::TestGetFluxVelocityInFrame() {

    auto velocity = flow->GetRelativeVelocityInFrame(m_frame, m_FrameVelocityInWorld, NWU);
    EXPECT_FLOAT_EQ(velocity.GetVx(), m_RelativeVelocityInFrame.GetVx());
    EXPECT_FLOAT_EQ(velocity.GetVy(), m_RelativeVelocityInFrame.GetVy());
    EXPECT_FLOAT_EQ(velocity.GetVz(), m_RelativeVelocityInFrame.GetVz());

    auto FrameVelocityInWorld_NED = Velocity(m_FrameVelocityInWorld.GetVx(),
                                             -m_FrameVelocityInWorld.GetVy(),
                                             -m_FrameVelocityInWorld.GetVz());
    velocity = flow->GetRelativeVelocityInFrame(m_frame, FrameVelocityInWorld_NED, NED);
    EXPECT_FLOAT_EQ(velocity.GetVx(), m_RelativeVelocityInFrame.GetVx());
    EXPECT_FLOAT_EQ(velocity.GetVy(), m_RelativeVelocityInFrame.GetVy());
    EXPECT_FLOAT_EQ(velocity.GetVz(), m_RelativeVelocityInFrame.GetVz());
}


TEST_F(TestFrFlowBase, GetFluxVelocityInWorld) {
    TestGetFluxVelocityInWorld();
}

TEST_F(TestFrFlowBase, GetRelativeVelocityInFrame) {
    TestGetFluxVelocityInFrame();
}

TEST_F(TestFrFlowBase, Update) {
    flow->Update(0.);
}

TEST_F(TestFrFlowBase, Initialize)  {
    flow->Initialize();
}

TEST_F(TestFrFlowBase, StepFinalize) {
    flow->StepFinalize();
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
