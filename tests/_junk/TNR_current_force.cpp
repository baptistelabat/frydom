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

    /**
    AssertIsClose(force.GetFx(), forceRef(0));
    AssertIsClose(force.GetFy(), forceRef(1));
    AssertIsClose(force.GetFz(), forceRef(2));
    AssertIsClose(torque.GetMx(), forceRef(3));
    AssertIsClose(torque.GetMy(), forceRef(4));
    AssertIsClose(torque.GetMz(), forceRef(5));
    **/

    std::cout << force.GetFx() << " compared to " << forceRef(0) << " : " << IsClose(force.GetFx(), forceRef(0)) << std::endl;
    std::cout << force.GetFy() << " compared to " << forceRef(1) << " : " << IsClose(force.GetFy(), forceRef(1)) << std::endl;
    std::cout << force.GetFz() << " compared to " << forceRef(2) << " : " << IsClose(force.GetFz(), forceRef(2)) << std::endl;
    std::cout << torque.GetMx() << " compared to " << forceRef(3) << " : " <<  IsClose(torque.GetMx(), forceRef(3)) << std::endl;
    std::cout << torque.GetMy() << " compared to " << forceRef(4) << " : " << IsClose(torque.GetMy(), forceRef(4)) << std::endl;
    std::cout << torque.GetMz() << " compared to " << forceRef(5) << " : " << IsClose(torque.GetMz(), forceRef(5)) << std::endl;
    return;

}

int main() {

    // System

    FrOffshoreSystem_ system;

    // ship

    auto ship = std::make_shared<FrBody_>();
    system.AddBody(ship);
    ship->SetAbsPosition(0., 0., 0., NWU);
    ship->SetCOGLocalPosition(0., 0. , 0.03, false, NWU);

    // Force

    auto current_force = std::make_shared<FrCurrentForce_>("../Ship_PolarCurrentCoeffs.yml");
    ship->AddExternalForce(current_force);

    // Initialize

    system.Initialize();

    // Loop

    FrHDF5Reader reader;

    reader.SetFilename("TNR_database.h5");
    std::string group = "/current_force/";

    auto current_speed = reader.ReadDoubleArray(group + "speed/");
    auto current_dir   = reader.ReadDoubleArray(group + "direction/");
    auto forceREF      = reader.ReadDoubleArray(group + "force/");

    auto angle_unit = AngleUnit[ reader.ReadString(group + "angle_unit/") ];
    auto speed_unit = SpeedUnit[ reader.ReadString(group + "speed_unit/") ];
    auto convention = DirConvention[ reader.ReadString(group + "convention/") ];
    auto frame = FrameConv[ reader.ReadString(group + "frame/") ];

    Force force;
    Moment torque;

    for (unsigned int i=0; i<current_speed.size(); i++) {

        std::cout << "i loop = " << i << std::endl;

        system.GetEnvironment()->GetCurrent()->GetField()->Set(current_dir(i), current_speed(i), angle_unit, speed_unit, frame, convention);
        //system.Initialize();
        current_force->Update(false);
        current_force->GetAbsForce(force, NWU);
        current_force->GetLocalTorqueAtCOG(torque, NWU);

        CompareForceValue(force, torque, forceREF.row(i));
    }

    return 0;
}