//
// Created by camille on 14/11/18.
//

#include "boost/assign/list_of.hpp"
#include "frydom/frydom.h"
#include "gtest/gtest.h"
//#include "frydom/core/FrVector.h"

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
// TEST OF THE UNIFORM FIELD
//
// --------------------------------------------------------------------

class TestFrUniformField : public ::testing::Test {

protected:

    FrUniformField field;

    double m_angle_RAD_NWU_GOTO;
    double m_angle_DEG_NWU_GOTO;
    double m_angle_RAD_NWU_COMEFROM;
    double m_angle_DEG_NWU_COMEFROM;
    double m_angle_RAD_NED_GOTO;
    double m_angle_DEG_NED_GOTO;
    double m_angle_RAD_NED_COMEFROM;
    double m_angle_DEG_NED_COMEFROM;

    double m_speed_MS;
    double m_speed_KMH;
    double m_speed_KNOT;

    Position m_PointInWorld;
    Velocity m_VelocityInWorld;

    void SetUp() override;

    void LoadData(std::string filename);

    template <class Vector>
    Vector ReadVector(FrHDF5Reader& reader, std::string field) const;

public:
    void TestSetVelocityFrameDir();
    void TestSetAngleSpeedFrameDir();
    void TestSetNorthSpeedDir();
    void TestSetNorthEastSpeedDir();
    void TestSetEastSpeedDir();
    void TestSetSouthEastSpeedDir();
    void TestSetSouthSpeedDir();
    void TestSetSouthWestSpeedDir();
    void TestSetWestSpeedDir();
    void TestSetNorthWestSpeedDir();
    void TestSpecificCase();

    void CheckField();

    void CheckField(Velocity vect);

    template <class Vector=Velocity>
    Vector SwapDirectionConvention(Vector vect);

    template <class Vector=Velocity>
    Vector SwapFrameConvention(Vector vect);

    double GetSpeedRef(SPEED_UNIT unit) const;
};

void TestFrUniformField::CheckField() {

    auto velocity = field.GetFluxVelocityInWorld(m_PointInWorld, NWU);
    EXPECT_NEAR(velocity.GetVx(), m_VelocityInWorld.GetVx(), 10E-4);
    EXPECT_NEAR(velocity.GetVy(), m_VelocityInWorld.GetVy(), 10E-4);
    EXPECT_NEAR(velocity.GetVz(), m_VelocityInWorld.GetVz(), 10E-4);
}

void TestFrUniformField::CheckField(Velocity vect) {

    auto velocity = field.GetFluxVelocityInWorld(m_PointInWorld, NWU);
    EXPECT_NEAR(velocity.GetVx(), vect.GetVx(), 10E-4);
    EXPECT_NEAR(velocity.GetVy(), vect.GetVy(), 10E-4);
    EXPECT_NEAR(velocity.GetVz(), vect.GetVz(), 10E-4);
}

template <class Vector>
Vector TestFrUniformField::ReadVector(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return Vector(value(0), value(1), value(2));
}

template <class Vector>
Vector TestFrUniformField::SwapDirectionConvention(Vector vect) {
    Vector result;
    result[0] = - vect[0];
    result[1] = - vect[1];
    result[2] = - vect[2];
    return result;
}

template <class Vector>
Vector TestFrUniformField::SwapFrameConvention(Vector vect) {
    Vector result;
    result[0] = vect[0];
    result[1] = -vect[1];
    result[2] = -vect[2];
    return result;
}

double TestFrUniformField::GetSpeedRef(SPEED_UNIT unit) const {

    switch (unit) {
        case MS:
            return m_speed_MS;
        case KMH:
            return m_speed_KMH;
        case KNOT:
            return m_speed_KNOT;
    }
    return 0.;

}

void TestFrUniformField::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "field/uniform/";

    m_angle_RAD_NWU_GOTO      = reader.ReadDouble(group + "angle_RAD_NWU_goto");
    m_angle_DEG_NWU_GOTO      = reader.ReadDouble(group + "angle_DEG_NWU_goto");
    m_angle_RAD_NWU_COMEFROM  = reader.ReadDouble(group + "angle_RAD_NWU_comefrom");
    m_angle_DEG_NWU_COMEFROM  = reader.ReadDouble(group + "angle_DEG_NWU_comefrom");
    m_angle_RAD_NED_GOTO      = reader.ReadDouble(group + "angle_RAD_NED_goto");
    m_angle_DEG_NED_GOTO      = reader.ReadDouble(group + "angle_DEG_NED_goto");
    m_angle_RAD_NED_COMEFROM  = reader.ReadDouble(group + "angle_RAD_NED_comefrom");
    m_angle_DEG_NED_COMEFROM  = reader.ReadDouble(group + "angle_DEG_NED_comefrom");

    m_speed_MS = reader.ReadDouble(group + "speed_MS");
    m_speed_KMH = reader.ReadDouble(group + "speed_KMH");
    m_speed_KNOT = reader.ReadDouble(group + "speed_KNOT");

    m_PointInWorld = ReadVector<Position>(reader, group + "PointInWorld/");
    m_VelocityInWorld = ReadVector<Velocity>(reader, group + "VelocityInWorld/");
}

void TestFrUniformField::SetUp() {
    LoadData("TNR_database.h5");
}

void TestFrUniformField::TestSetVelocityFrameDir() {

    field.Set(m_VelocityInWorld, NWU, GOTO);
    CheckField();

    Velocity velocity, velocity2;

    velocity = this->SwapDirectionConvention(m_VelocityInWorld);
    field.Set(velocity, NWU, COMEFROM);
    CheckField();

    velocity = this->SwapFrameConvention(m_VelocityInWorld);
    field.Set(velocity, NED, GOTO);
    CheckField();

    velocity = this->SwapDirectionConvention(m_VelocityInWorld);
    velocity2 = this->SwapFrameConvention(velocity);
    field.Set(velocity2, NED, COMEFROM);
    CheckField();

}

void TestFrUniformField::TestSetAngleSpeedFrameDir() {

    double speed;

    for (auto uspeed: {MS, KMH, KNOT}) {

        speed = GetSpeedRef(uspeed);

        field.Set(m_angle_RAD_NWU_GOTO, speed, RAD, uspeed, NWU, GOTO);
        CheckField();
        field.Set(m_angle_DEG_NWU_GOTO, speed, DEG, uspeed, NWU, GOTO);
        CheckField();
        field.Set(m_angle_RAD_NWU_COMEFROM, speed, RAD, uspeed, NWU, COMEFROM);
        CheckField();
        field.Set(m_angle_DEG_NWU_COMEFROM, speed, DEG, uspeed, NWU, COMEFROM);
        CheckField();
        field.Set(m_angle_RAD_NED_GOTO, speed, RAD, uspeed, NED, GOTO);
        CheckField();
        field.Set(m_angle_DEG_NED_GOTO, speed, DEG, uspeed, NED, GOTO);
        CheckField();
        field.Set(m_angle_RAD_NED_COMEFROM, speed, RAD, uspeed, NED, COMEFROM);
        CheckField();
        field.Set(m_angle_DEG_NED_COMEFROM, speed, DEG, uspeed, NED, COMEFROM);
        CheckField();

    }
}


void TestFrUniformField::TestSetNorthSpeedDir() {

    double speed;

    for (auto uspeed: {MS, KNOT, KMH}) {

        speed = GetSpeedRef(uspeed);

        field.SetNorth(speed, uspeed, GOTO);
        Velocity velocityREF = {m_speed_MS, 0., 0.};
        CheckField(velocityREF);

        field.Set(NORTH, speed, uspeed, GOTO);
        velocityREF = {m_speed_MS, 0., 0.};
        CheckField(velocityREF);

        field.SetNorth(speed, uspeed, COMEFROM);
        velocityREF = {-m_speed_MS, 0., 0.};
        CheckField(velocityREF);

        field.Set(NORTH, speed, uspeed, COMEFROM);
        velocityREF = {-m_speed_MS, 0., 0.};
        CheckField(velocityREF);
    }
}

void TestFrUniformField::TestSetNorthEastSpeedDir() {

    double speed;

    for (auto uspeed: {MS, KNOT, KMH}) {
        speed = GetSpeedRef(uspeed);

        field.SetNorthEast(speed, uspeed, GOTO);
        Velocity velocityREF = {m_speed_MS * MU_SQRT2_2, -m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.Set(NORTH_EAST, speed, uspeed, GOTO);
        velocityREF = {m_speed_MS * MU_SQRT2_2, -m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.SetNorthEast(speed, uspeed, COMEFROM);
        velocityREF = {-m_speed_MS * MU_SQRT2_2, m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.Set(NORTH_EAST, speed, uspeed, COMEFROM);
        velocityREF = {-m_speed_MS * MU_SQRT2_2, m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);
    }
}

void TestFrUniformField::TestSetEastSpeedDir() {

    double speed;

    for (auto uspeed: {MS, KNOT, KMH}) {

        speed = GetSpeedRef(uspeed);

        field.SetEast(speed, uspeed, GOTO);
        Velocity velocityREF = {0., -m_speed_MS, 0.};
        CheckField(velocityREF);

        field.Set(EAST, speed, uspeed, GOTO);
        velocityREF = {0., -m_speed_MS, 0.};
        CheckField(velocityREF);

        field.SetEast(speed, uspeed, COMEFROM);
        velocityREF = {0., m_speed_MS, 0.};
        CheckField(velocityREF);

        field.Set(EAST, speed, uspeed, COMEFROM);
        velocityREF = {0., m_speed_MS, 0.};
        CheckField(velocityREF);
    }
}

void TestFrUniformField::TestSetSouthEastSpeedDir() {

    double speed;

    for (auto uspeed: {MS, KNOT, KMH}) {

        speed = GetSpeedRef(uspeed);

        field.SetSouthEast(speed, uspeed, GOTO);
        Velocity velocityREF = {-m_speed_MS * MU_SQRT2_2, -m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.Set(SOUTH_EAST, speed, uspeed, GOTO);
        velocityREF = {-m_speed_MS * MU_SQRT2_2, -m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.SetSouthEast(speed, uspeed, COMEFROM);
        velocityREF = {m_speed_MS * MU_SQRT2_2, m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.Set(SOUTH_EAST, speed, uspeed, COMEFROM);
        velocityREF = {m_speed_MS * MU_SQRT2_2, m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);
    }
}

void TestFrUniformField::TestSetSouthSpeedDir() {

    double speed;

    for (auto uspeed: {MS, KNOT, KMH}) {

        speed = GetSpeedRef(uspeed);

        field.SetSouth(speed, uspeed, GOTO);
        Velocity velocityREF = {-m_speed_MS, 0., 0.};
        CheckField(velocityREF);

        field.Set(SOUTH, speed, uspeed, GOTO);
        velocityREF = {-m_speed_MS, 0., 0.};
        CheckField(velocityREF);

        field.SetSouth(speed, uspeed, COMEFROM);
        velocityREF = {m_speed_MS, 0., 0.};
        CheckField(velocityREF);

        field.Set(SOUTH, speed, uspeed, COMEFROM);
        velocityREF = {m_speed_MS, 0., 0.};
        CheckField(velocityREF);
    }
}

void TestFrUniformField::TestSetSouthWestSpeedDir() {

    double speed;

    for (auto uspeed: {MS, KNOT, KMH}) {

        speed = GetSpeedRef(uspeed);

        field.SetSouthWest(speed, uspeed, GOTO);
        Velocity velocityREF = {-m_speed_MS * MU_SQRT2_2, m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.Set(SOUTH_WEST, speed, uspeed, GOTO);
        velocityREF = {-m_speed_MS * MU_SQRT2_2, m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.SetSouthWest(speed, uspeed, COMEFROM);
        velocityREF = {m_speed_MS * MU_SQRT2_2, -m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.Set(SOUTH_WEST, speed, uspeed, COMEFROM);
        velocityREF = {m_speed_MS * MU_SQRT2_2, -m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);
    }
}

void TestFrUniformField::TestSetWestSpeedDir() {

    double speed;

    for (auto uspeed: {MS, KNOT, KMH}) {

        speed = GetSpeedRef(uspeed);

        field.SetWest(speed, uspeed, GOTO);
        Velocity velocityREF = {0., m_speed_MS, 0.};
        CheckField(velocityREF);

        field.Set(WEST, speed, uspeed, GOTO);
        velocityREF = {0., m_speed_MS, 0.};
        CheckField(velocityREF);

        field.SetWest(speed, uspeed, COMEFROM);
        velocityREF = {0., -m_speed_MS, 0.};
        CheckField(velocityREF);

        field.Set(WEST, speed, uspeed, COMEFROM);
        velocityREF = {0., -m_speed_MS, 0.};
        CheckField(velocityREF);
    }
}

void TestFrUniformField::TestSetNorthWestSpeedDir() {

    double speed;

    for (auto uspeed: {MS, KNOT, KMH}) {

        speed = GetSpeedRef(uspeed);

        field.SetNorthWest(speed, uspeed, GOTO);
        Velocity velocityREF = {m_speed_MS * MU_SQRT2_2, m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.Set(NORTH_WEST, speed, uspeed, GOTO);
        velocityREF = {m_speed_MS * MU_SQRT2_2, m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.SetNorthWest(speed, uspeed, COMEFROM);
        velocityREF = {-m_speed_MS * MU_SQRT2_2, -m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);

        field.Set(NORTH_WEST, speed, uspeed, COMEFROM);
        velocityREF = {-m_speed_MS * MU_SQRT2_2, -m_speed_MS * MU_SQRT2_2, 0.};
        CheckField(velocityREF);
    }
}

void TestFrUniformField::TestSpecificCase() {

    field.Set(m_angle_RAD_NWU_GOTO + 2.*M_PI, m_speed_MS, RAD, MS, NWU, GOTO);
    CheckField();

    field.Set(-m_angle_RAD_NED_GOTO, m_speed_MS, RAD, MS, NWU, GOTO);
    CheckField();

}

TEST_F(TestFrUniformField, SetVelocityFrameDir) {
    TestSetVelocityFrameDir();
}

TEST_F(TestFrUniformField, SetAngleSpeedFrameDir) {
    TestSetAngleSpeedFrameDir();
}

TEST_F(TestFrUniformField, SetNorthSpeedDir) {
    TestSetNorthSpeedDir();
}

TEST_F(TestFrUniformField, SetNorthEastSpeedDir) {
    TestSetNorthEastSpeedDir();
}

TEST_F(TestFrUniformField, SetEastSpeedDir) {
    TestSetEastSpeedDir();
}

TEST_F(TestFrUniformField, SetSouthEastSpeedDir) {
    TestSetSouthEastSpeedDir();
}

TEST_F(TestFrUniformField, SetSouthSpeedDir) {
    TestSetSouthSpeedDir();
}
TEST_F(TestFrUniformField, SetSouthWestSpeedDir) {
    TestSetSouthWestSpeedDir();
}
TEST_F(TestFrUniformField, SetWestSpeedDir) {
    TestSetWestSpeedDir();
}
TEST_F(TestFrUniformField, SetNorthWestSpeedDir) {
    TestSetNorthWestSpeedDir();
}

TEST_F(TestFrUniformField, SpecificCase) {
    TestSpecificCase();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}