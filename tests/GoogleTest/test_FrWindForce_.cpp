//
// Created by camille on 07/11/18.
//

#include "boost/assign/list_of.hpp"
#include "frydom/frydom.h"
#include "gtest/gtest.h"


using namespace frydom;

// -------------------------------------------------------------------
//
// MAP TO HELP UNIT CONVERSION
//
// -------------------------------------------------------------------


std::map<std::string, ANGLE_UNIT>
        AngleUnit = boost::assign::map_list_of("DEG", DEG)("RAD", RAD);

std::map<std::string, SPEED_UNIT>
        SpeedUnit = boost::assign::map_list_of("MS", MS)("KNOT", KNOT)("KMH", KMH);

std::map<std::string, FRAME_CONVENTION>
        FrameConv = boost::assign::map_list_of("NWU", NWU)("NED", NED);

std::map<std::string, DIRECTION_CONVENTION>
        DirConvention = boost::assign::map_list_of("GOTO", GOTO)("COMEFROM", COMEFROM);


// --------------------------------------------------------------------
//
// TEST OF THE UNIFORM CURRENT FIELD
//
// --------------------------------------------------------------------

class TestFrUniformWind_ : public ::testing::Test {

protected:

    FrOffshoreSystem_ system;                   ///< offshore system

    double m_angle;                             ///< wind direction
    double m_speed;                             ///< wind speed
    ANGLE_UNIT m_angleUnit;                     ///< wind direction unit (RAD/DEG)
    SPEED_UNIT m_speedUnit;                     ///< wind speed unit (KNOT/MS)
    FRAME_CONVENTION m_frame;                   ///< frame convention (NED/NWU)
    DIRECTION_CONVENTION m_convention;          ///< direction convention (GOTO/COMEFROM)

    Position m_PointInWorld;                    ///< Position of an arbitrary point in world frame
    FrFrame_ m_frameREF;                        ///< Local frame at Point
    FrQuaternion_ m_quatREF;                    ///< Rotation of the local frame / world frame
    Velocity m_PointVelocityInWorld;            ///< Velocity vector of the local frame / world frame

    Velocity m_VelocityInWorld;                 ///< Wind velocity at Point in world frame
    Velocity m_RelativeVelocityInWorld;         ///< Relative velocity at Point in world frame
    Velocity m_RelativeVelocityInFrame;         ///< Relative velocity at Point in local frame

    /// Initialization of the method
    void SetUp() override;

    /// Loading data from HDF5 file
    void LoadData(std::string filename);

    /// Vector reading method
    template <class Vector>
    Vector ReadVector(FrHDF5Reader& reader, std::string field) const;

public:
    /// Test of the get flux vector method
    void TestGetWorldFluxVelocity();

    /// Test of the get relative velocity in frame method
    void TestGetRelativeVelocityInFrame();

};

template <class Vector>
Vector TestFrUniformWind_::ReadVector(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return Vector(value(0), value(1), value(2));
}

void TestFrUniformWind_::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "wind/uniform/";

    m_angle = reader.ReadDouble(group + "angle/");
    m_speed = reader.ReadDouble(group + "speed/");
    m_angleUnit = AngleUnit[ reader.ReadString(group + "angle_unit/")];
    m_speedUnit = SpeedUnit[ reader.ReadString(group + "speed_unit/")];
    m_frame = FrameConv[ reader.ReadString(group + "frame_convention/")];
    m_convention = DirConvention[ reader.ReadString(group + "direction_convention/")];

    m_PointInWorld = ReadVector<Position>(reader, group + "PointInWorld");
    auto direction = ReadVector<Direction>(reader, group + "RotationDirection/") ;
    double angle = reader.ReadDouble(group + "RotationAngle/");
    m_quatREF = FrQuaternion_(direction, angle, NWU);
    m_frameREF = FrFrame_(m_PointInWorld, m_quatREF, NWU);
    m_PointVelocityInWorld = ReadVector<Velocity>(reader, group + "PointVelocityInWorld/");

    m_VelocityInWorld = ReadVector<Velocity>(reader, group + "VelocityInWorld/");
    m_RelativeVelocityInWorld = ReadVector<Velocity>(reader, group + "RelativeVelocityInWorld");
    m_RelativeVelocityInFrame = ReadVector<Velocity>(reader, group + "RelativeVelocityInFrame");

}

void TestFrUniformWind_::SetUp() {

    LoadData("TNR_database.h5");
    system.GetEnvironment()->GetWind()->GetFieldUniform()->Set(m_angle, m_speed, m_angleUnit, m_speedUnit, m_frame, m_convention);
}

void TestFrUniformWind_::TestGetWorldFluxVelocity() {

    Velocity velocity = system.GetEnvironment()->GetWind()->GetFluxVelocityInWorld(m_PointInWorld, m_frame);
    Velocity velocityREF = velocity - m_PointVelocityInWorld;

    EXPECT_FLOAT_EQ(velocity.GetVx(), m_VelocityInWorld.GetVx());
    EXPECT_FLOAT_EQ(velocity.GetVy(), m_VelocityInWorld.GetVy());
    EXPECT_FLOAT_EQ(velocity.GetVz(), m_VelocityInWorld.GetVz());
}

void TestFrUniformWind_::TestGetRelativeVelocityInFrame() {

    Velocity velocity = system.GetEnvironment()->GetWind()->GetRelativeVelocityInFrame(m_frameREF, m_PointVelocityInWorld, m_frame);

    EXPECT_FLOAT_EQ(velocity.GetVx(), m_RelativeVelocityInFrame.GetVx());
    EXPECT_FLOAT_EQ(velocity.GetVy(), m_RelativeVelocityInFrame.GetVy());
    EXPECT_FLOAT_EQ(velocity.GetVz(), m_RelativeVelocityInFrame.GetVz());
}


TEST_F(TestFrUniformWind_, Update) {
    system.GetEnvironment()->GetWind()->Update(0.);
}

TEST_F(TestFrUniformWind_, Initialize) {
    system.GetEnvironment()->GetWind()->Initialize();
}

TEST_F(TestFrUniformWind_, StepFinalize) {
    system.GetEnvironment()->GetWind()->StepFinalize();
}

TEST_F(TestFrUniformWind_, GetField) {
    auto field = system.GetEnvironment()->GetWind()->GetFieldUniform();
}

TEST_F(TestFrUniformWind_, TestWorldFluxVelocity) {
    TestGetWorldFluxVelocity();
}

TEST_F(TestFrUniformWind_, TestRelativeFluxVelocity) {
    TestGetRelativeVelocityInFrame();
}

// ---------------------------------------------------------------------------
//
// TEST OF THE CURRENT FORCE OBJECT
//
// ----------------------------------------------------------------------------

class TestFrWindForce_ : public testing::Test {

protected:

    FrOffshoreSystem_ system;                               ///< offshore system
    std::shared_ptr<FrBody_> body;                          ///< hydrodynamic body
    std::shared_ptr<FrWindForce_> force;                 ///< wind force

    const Position bodyPositionInWorld = Position(0., 0., 0.);  ///< Position of Point in world
    const Position COGPosition = Position(0., 0., 0.03);        ///< Position of the COG in body

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> wind_speed;  ///< List of wind speed test
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> wind_dir;    ///< List of wind direction test
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> forceREF;       ///< List of force results for the test

    ANGLE_UNIT angleUnit;                                   ///< current direction unit (RAD/DEG)
    SPEED_UNIT speedUnit;                                   ///< current speed unit (KNOT/MS)
    FRAME_CONVENTION frame;                                 ///< frame convention (NED/NWU)
    DIRECTION_CONVENTION convention;                        ///< direction convention (GOTO/COMEFROM)

    /// Initialize environment
    void SetUp() override;

    /// Loading data from HDF5 file
    void LoadData(std::string filename);

public:
    /// Test the current force vector
    void TestForce();

    /// Compare the force value in world at the COG
    void CheckForceInWorldAtCOG(Force force, const unsigned int index);

    /// Compare the torque value in body at the COG
    void CheckTorqueInBodyAtCOG(Torque torque, const unsigned int index);

};

void TestFrWindForce_::SetUp() {

    this->LoadData("TNR_database.h5");

    body = std::make_shared<FrBody_>();
    body->SetPosition(bodyPositionInWorld, NWU);
    body->SetCOG(COGPosition, NWU);
    system.AddBody(body);

    force = std::make_shared<FrWindForce_>("../Ship_PolarWindCoeffs.yml");
    body->AddExternalForce(force);

    system.Initialize();
}

void TestFrWindForce_::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "/wind_force/";

    wind_speed = reader.ReadDoubleArray(group + "speed/");
    wind_dir   = reader.ReadDoubleArray(group + "direction/");
    forceREF      = reader.ReadDoubleArray(group + "force/");

    angleUnit = AngleUnit[ reader.ReadString(group + "angle_unit/") ];
    speedUnit = SpeedUnit[ reader.ReadString(group + "speed_unit/") ];
    convention = DirConvention[ reader.ReadString(group + "convention/") ];
    frame = FrameConv[ reader.ReadString(group + "frame/") ];
}

void TestFrWindForce_::CheckForceInWorldAtCOG(Force force, const unsigned int index) {

    auto forceRef_i = forceREF.row(index);
    EXPECT_FLOAT_EQ(force.GetFx(), forceRef_i(0));
    EXPECT_FLOAT_EQ(force.GetFy(), forceRef_i(1));
    EXPECT_FLOAT_EQ(force.GetFz(), forceRef_i(2));
}

void TestFrWindForce_::CheckTorqueInBodyAtCOG(Torque torque, const unsigned int index) {

    auto forceRef_i = forceREF.row(index);
    EXPECT_FLOAT_EQ(torque.GetMx(), forceRef_i(3));
    EXPECT_FLOAT_EQ(torque.GetMy(), forceRef_i(4));
    EXPECT_FLOAT_EQ(torque.GetMz(), forceRef_i(5));
}

void TestFrWindForce_::TestForce() {
    Force forceTemp;
    Torque torqueTemp;

    for (unsigned int i=0; i<wind_speed.size(); i++) {
        system.GetEnvironment()->GetWind()->GetFieldUniform()->Set(wind_dir(i), wind_speed(i), angleUnit, speedUnit, frame, convention);
        force->Update(false);
        force->GetForceInWorld(forceTemp, NWU);
        force->GetTorqueInBodyAtCOG(torqueTemp, NWU);

        CheckForceInWorldAtCOG(forceTemp, i);
        CheckTorqueInBodyAtCOG(torqueTemp, i);
    }
}

TEST_F(TestFrWindForce_, TestForce) {
    TestForce();
};


