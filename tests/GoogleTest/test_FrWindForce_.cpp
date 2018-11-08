//
// Created by camille on 07/11/18.
//

#include "boost/assign/list_of.hpp"
#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;

std::map<std::string, ANGLE_UNIT>
        AngleUnit = boost::assign::map_list_of("DEG", DEG)("RAD", RAD);

std::map<std::string, SPEED_UNIT>
        SpeedUnit = boost::assign::map_list_of("MS", MS)("KNOT", KNOT)("KMH", KMH);

std::map<std::string, FRAME_CONVENTION>
        FrameConv = boost::assign::map_list_of("NWU", NWU)("NED", NED);

std::map<std::string, DIRECTION_CONVENTION>
        DirConvention = boost::assign::map_list_of("GOTO", GOTO)("COMEFROM", COMEFROM);


void CompareForceValue(Force force, Moment torque,
                       Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> forceRef) {

    EXPECT_FLOAT_EQ(force.GetFx(), forceRef(0));
    EXPECT_FLOAT_EQ(force.GetFy(), forceRef(1));
    EXPECT_FLOAT_EQ(force.GetFz(), forceRef(2));
    EXPECT_FLOAT_EQ(torque.GetMx(), forceRef(3));
    EXPECT_FLOAT_EQ(torque.GetMy(), forceRef(4));
    EXPECT_FLOAT_EQ(torque.GetMz(), forceRef(5));
}

TEST(FrWindForce_test, ALL) {

    // System

    FrOffshoreSystem_ system;

    // Ship

    auto ship = std::make_shared<FrBody_>();
    system.AddBody(ship);
    ship->SetAbsPosition(0., 0., 0., NWU);
    ship->SetCOGLocalPosition(0., 0. , 0.03, false, NWU);

    // Force

    auto wind_force = std::make_shared<FrWindForce_>("../Ship_PolarWindCoeffs.yml");
    ship->AddExternalForce(wind_force);

    // Initialize

    system.Initialize();

    // Loop

    FrHDF5Reader reader;

    reader.SetFilename("TNR_database.h5");
    std::string group = "/wind_force/";

    auto speed = reader.ReadDoubleArray(group + "speed/");
    auto dir   = reader.ReadDoubleArray(group + "direction/");
    auto forceREF      = reader.ReadDoubleArray(group + "force/");

    auto angle_unit = AngleUnit[ reader.ReadString(group + "angle_unit/") ];
    auto speed_unit = SpeedUnit[ reader.ReadString(group + "speed_unit/") ];
    auto convention = DirConvention[ reader.ReadString(group + "convention/") ];
    auto frame = FrameConv[ reader.ReadString(group + "frame/") ];

    Force force;
    Moment torque;

    for (unsigned int i=0; i<speed.size(); i++) {

        system.GetEnvironment()->GetWind()->GetField()->Set(dir(i), speed(i), angle_unit, speed_unit, frame, convention);
        wind_force->Update(false);
        wind_force->GetAbsForce(force, NWU);
        wind_force->GetLocalTorqueAtCOG(torque, NWU);

        CompareForceValue(force, torque, forceREF.row(i));
    }

};
