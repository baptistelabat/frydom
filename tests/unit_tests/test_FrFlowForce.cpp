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



// ---------------------------------------------------------------------------
//
// TEST OF THE FLOW FORCE OBJECT
//
// ----------------------------------------------------------------------------

class TestFrFlowForce : public testing::Test {


protected:

    FrOffshoreSystem_ system;                                       ///< offshore system
    std::shared_ptr<FrBody_> body;                                  ///< hydrodynamic body
    std::shared_ptr<FrFlowForce> force;                            ///< flow force

    const Position bodyPositionInWorld = Position(0., 0., 0.);      ///< Position of Point in world
    const Position COGPosition = Position(0., 0., 0.03);            ///< Position of the COG in body

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> speed;    ///< List of speed test
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dir;      ///< List of direction test
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> forceREF; ///< List of force results for the test

    ANGLE_UNIT angleUnit;                                           ///< direction unit (RAD/DEG)
    SPEED_UNIT speedUnit;                                           ///< speed unit (KNOT/MS)
    FRAME_CONVENTION frame;                                         ///< frame convention (NED/NWU)
    DIRECTION_CONVENTION convention;                                ///< direction convention (GOTO/COMEFROM)

    FLUID_TYPE m_type;

    /// Initialize environment
    void SetUp() override;


public:
    /// Test the force vector
    void TestForce();

    /// Compare the force value in world at the COG
    void CheckForceInWorldAtCOG(Force force, const unsigned int index);

    /// Compare the torque value in body at the COG
    void CheckTorqueInBodyAtCOG(Torque torque, const unsigned int index);

    /// Loading data from HDF5 file
    void LoadData(std::string filename, std::string group);

    ///
    void MakeForce(FLUID_TYPE type, std::string filename);

};

void TestFrFlowForce::SetUp() {
    body = std::make_shared<FrBody_>();
    body->SetPosition(bodyPositionInWorld, NWU);

    FrInertiaTensor_ InertiaTensor(1.,1.,1.,1.,0.,0.,0.,FrFrame_(COGPosition,FrRotation_(),NWU),NWU);
    body->SetInertiaTensor(InertiaTensor);

    system.AddBody(body);
}

void TestFrFlowForce::LoadData(std::string filename, std::string group) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);

    speed = reader.ReadDoubleArray(group + "speed/");
    dir   = reader.ReadDoubleArray(group + "direction/");
    forceREF      = reader.ReadDoubleArray(group + "force/");

    angleUnit = STRING2ANGLE( reader.ReadString(group + "angle_unit/") );
    speedUnit = SpeedUnit[ reader.ReadString(group + "speed_unit/") ];
    convention = DirConvention[ reader.ReadString(group + "convention/") ];
    frame = FrameConv[ reader.ReadString(group + "frame/") ];
}

void TestFrFlowForce::MakeForce(FLUID_TYPE type, std::string filename) {
    m_type = type;
    if (type==FLUID_TYPE::WATER) {
        force = std::make_shared<FrCurrentForce2_>(filename);
    } else if (type==FLUID_TYPE::AIR) {
        force = std::make_shared<FrWindForce2_>(filename);
    }
    body->AddExternalForce(force);
}

void TestFrFlowForce::CheckForceInWorldAtCOG(Force force, const unsigned int index) {

    auto forceRef_i = forceREF.row(index);
    EXPECT_NEAR(forceRef_i(0), force.GetFx(), 10e-2);
    EXPECT_NEAR(forceRef_i(1), force.GetFy(), 10e-2);
    EXPECT_NEAR(forceRef_i(2), force.GetFz(), 10e-2);
}

void TestFrFlowForce::CheckTorqueInBodyAtCOG(Torque torque, const unsigned int index) {

    auto forceRef_i = forceREF.row(index);
    EXPECT_NEAR(forceRef_i(3), torque.GetMx(), 10e-2);
    EXPECT_NEAR(forceRef_i(4), torque.GetMy(), 10e-2);
    EXPECT_NEAR(forceRef_i(5), torque.GetMz(), 10e-2);
}

void TestFrFlowForce::TestForce() {
    Force forceTemp;
    Torque torqueTemp;

    for (unsigned int i=0; i<speed.size(); i++) {

        if (m_type == WATER) {
            system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
            system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(dir(i), speed(i), angleUnit, speedUnit, frame,
                                                                   convention);
        } else if (m_type == FLUID_TYPE::AIR) {
            system.GetEnvironment()->GetAtmosphere()->GetWind()->MakeFieldUniform();
            system.GetEnvironment()->GetAtmosphere()->GetWind()->GetFieldUniform()->Set(dir(i), speed(i), angleUnit, speedUnit, frame,
                                                                   convention);
        }
        force->Update(false);
        force->GetForceInWorld(forceTemp, NWU);
        force->GetTorqueInBodyAtCOG(torqueTemp, NWU);

        CheckForceInWorldAtCOG(forceTemp, i);
        CheckTorqueInBodyAtCOG(torqueTemp, i);
    }
}


TEST_F(TestFrFlowForce, TestCurrentForce) {
    LoadData("TNR_database.h5", "/current_force/");
    MakeForce(WATER, "../Ship_PolarCurrentCoeffs.yml");
    system.Initialize();
    TestForce();
};


TEST_F(TestFrFlowForce, TestWindForce) {
    LoadData("TNR_database.h5", "/wind_force/");
    MakeForce(FLUID_TYPE::AIR, "../Ship_PolarWindCoeffs.yml");
    system.Initialize();
    TestForce();
};

//TEST_F(TestFrFlowForce, TestFlowForce) {
//    LoadData("TNR_database.h5", "/current_force/");
//    //MakeForce(FLUID_TYPE::WATER, "Ship_polarTest_NWU_COMEFROM_0_PI.yml"); // OK
//    //MakeForce(FLUID_TYPE::WATER, "Ship_polarTest_NWU_COMEFROM_0_180.yml"); // OK
//    //MakeForce(FLUID_TYPE::WATER, "Ship_polarTest_NWU_COMEFROM_0_2PI.yml"); // OK (numerical error)
//    MakeForce(FLUID_TYPE::WATER, "Ship_polarTest_NWU_GOTO_0_PI.yml"); // OK (numerical error)
//    //MakeForce(FLUID_TYPE::WATER, "Ship_polarTest_NWU_COMEFROM__PI_PI.yml"); // NO
//    //MakeForce(FLUID_TYPE::WATER, "Ship_polarTest_NED_COMEFROM_0_PI.yml"); // OK (numerical error)
//    system.Initialize();
//    TestForce();
//};


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}