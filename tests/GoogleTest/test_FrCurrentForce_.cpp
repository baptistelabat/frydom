//
// Created by camille on 07/11/18.
//

#include "boost/assign/list_of.hpp"
#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;

//
//
// MAP TO HELP UNIT CONVERSION
//
//


std::map<std::string, ANGLE_UNIT>
        AngleUnit = boost::assign::map_list_of("DEG", DEG)("RAD", RAD);

std::map<std::string, SPEED_UNIT>
        SpeedUnit = boost::assign::map_list_of("MS", MS)("KNOT", KNOT)("KMH", KMH);

std::map<std::string, FRAME_CONVENTION>
        FrameConv = boost::assign::map_list_of("NWU", NWU)("NED", NED);

std::map<std::string, DIRECTION_CONVENTION>
        DirConvention = boost::assign::map_list_of("GOTO", GOTO)("COMEFROM", COMEFROM);


//
//
// TEST OF THE UNIFORM CURRENT FIELD
//
//

class TestFrUniformCurrent_ : public ::testing::Test {

protected:

    /// Environment objects
    FrOffshoreSystem_ system;

    /// Initialize environment
    void SetUp() override;
    void LoadData(std::string filename);

    template <class Vector>
    Vector ReadVector(FrHDF5Reader& reader, std::string field) const;

    /// Dataset
    double m_angle;
    double m_speed;
    ANGLE_UNIT m_angleUnit;
    SPEED_UNIT m_speedUnit;
    FRAME_CONVENTION m_frame;
    DIRECTION_CONVENTION m_convention;
    Velocity m_fluxVector;

    Position m_PointInWorld;
    FrQuaternion_ m_quatREF;
    FrFrame_ m_frameREF;
    Velocity m_PointVelocityInWorld;


public:
    /// List of tests
    void TestGetWorldFluxVelocity();
    void TestGetRelativeVelocityInFrame();
    void TestUpdate();

};

template <class Vector>
Vector TestFrUniformCurrent_::ReadVector(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return Vector(value(0), value(1), value(2));
}

void TestFrUniformCurrent_::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "current/uniform/";

    m_angle = reader.ReadDouble(group + "angle/");
    m_speed = reader.ReadDouble(group + "speed/");
    m_angleUnit = AngleUnit[ reader.ReadString(group + "angle_unit/")];
    m_speedUnit = SpeedUnit[ reader.ReadString(group + "speed_unit/")];
    m_frame = FrameConv[ reader.ReadString(group + "frame_convention/")];
    m_convention = DirConvention[ reader.ReadString(group + "direction_convention/")];

    auto value = reader.ReadDoubleArray(group + "flux_vector/");
    m_fluxVector = Velocity(value(0), value(1), value(2));

    m_PointInWorld = ReadVector<Position>(reader, group + "PointInWorld");
    auto direction = ReadVector<Direction>(reader, group + "RotationDirection/") ;
    double angle = reader.ReadDouble(group + "RotationAngle/");
    m_quatREF = FrQuaternion_(direction, angle, NWU);
    m_frameREF = FrFrame_(m_PointInWorld, m_quatREF, NWU);

    m_PointVelocityInWorld = ReadVector<Velocity>(reader, group + "PointVelocityInWorld/");

}

void TestFrUniformCurrent_::SetUp() {

    LoadData("TNR_database.h5");
    system.GetEnvironment()->GetCurrent()->GetField()->Set(m_angle, m_speed, m_angleUnit, m_speedUnit, m_frame, m_convention);

}

void TestFrUniformCurrent_::TestGetWorldFluxVelocity() {
    // TODO
}

void TestFrUniformCurrent_::TestGetRelativeVelocityInFrame() {
    // TODO
}

void TestFrUniformCurrent_::TestUpdate() {
    // TODO
}


//
//
// TEST OF THE CURRENT FORCE OBJECT
//
//

class TestFrCurrentForce_ : public testing::Test {

protected:
    /// Environment
    FrOffshoreSystem_ system;
    std::shared_ptr<FrBody_> body;
    std::shared_ptr<FrCurrentForce_> force;

    /// Dataset
    const Position bodyPositionInWorld = Position(0., 0., 0.);
    const Position COGPosition = Position(0., 0., 0.03);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> current_speed;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> current_dir;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> forceREF;

    ANGLE_UNIT angleUnit;
    SPEED_UNIT speedUnit;
    FRAME_CONVENTION frame;
    DIRECTION_CONVENTION convention;

    /// Initialize environment
    void SetUp() override;
    void LoadData(std::string filename);

public:
    void TestForce();
    void CheckForceInWorldAtCOG(Force force, const unsigned int index);
    void CheckTorqueInBodyAtCOG(Torque torque, const unsigned int index);

};

void TestFrCurrentForce_::SetUp() {

    this->LoadData("TNR_database.h5");

    body = std::make_shared<FrBody_>();
    body->SetPosition(bodyPositionInWorld, NWU);
    body->SetCOG(COGPosition, NWU);
    system.AddBody(body);

    force = std::make_shared<FrCurrentForce_>("../Ship_PolarCurrentCoeffs.yml");
    body->AddExternalForce(force);

    system.Initialize();
}

void TestFrCurrentForce_::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "/current_force/";

    current_speed = reader.ReadDoubleArray(group + "speed/");
    current_dir   = reader.ReadDoubleArray(group + "direction/");
    forceREF      = reader.ReadDoubleArray(group + "force/");

    angleUnit = AngleUnit[ reader.ReadString(group + "angle_unit/") ];
    speedUnit = SpeedUnit[ reader.ReadString(group + "speed_unit/") ];
    convention = DirConvention[ reader.ReadString(group + "convention/") ];
    frame = FrameConv[ reader.ReadString(group + "frame/") ];
}

void TestFrCurrentForce_::CheckForceInWorldAtCOG(Force force, const unsigned int index) {

    auto forceRef_i = forceREF.row(index);
    EXPECT_FLOAT_EQ(force.GetFx(), forceRef_i(0));
    EXPECT_FLOAT_EQ(force.GetFy(), forceRef_i(1));
    EXPECT_FLOAT_EQ(force.GetFz(), forceRef_i(2));
}

void TestFrCurrentForce_::CheckTorqueInBodyAtCOG(Torque torque, const unsigned int index) {

    auto forceRef_i = forceREF.row(index);
    EXPECT_FLOAT_EQ(torque.GetMx(), forceRef_i(3));
    EXPECT_FLOAT_EQ(torque.GetMy(), forceRef_i(4));
    EXPECT_FLOAT_EQ(torque.GetMz(), forceRef_i(5));
}

void TestFrCurrentForce_::TestForce() {
    Force forceTemp;
    Torque torqueTemp;

    for (unsigned int i=0; i<current_speed.size(); i++) {
        system.GetEnvironment()->GetCurrent()->GetField()->Set(current_dir(i), current_speed(i), angleUnit, speedUnit, frame, convention);
        force->Update(false);
        force->GetForceInWorld(forceTemp, NWU);
        force->GetTorqueInBodyAtCOG(torqueTemp, NWU);

        CheckForceInWorldAtCOG(forceTemp, i);
        CheckTorqueInBodyAtCOG(torqueTemp, i);
    }
}

TEST_F(TestFrCurrentForce_, TestForce) {
    TestForce();
};


