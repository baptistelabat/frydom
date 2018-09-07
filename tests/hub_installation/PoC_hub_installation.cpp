//
// Created by Lucas Letournel on 06/09/18.
//
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include <chrono/physics/ChLinkMotorRotationTorque.h>
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "frydom/frydom.h"
#include "PoC_hub_installation_makers.cpp"

using namespace chrono;
using namespace frydom;

int main(int argc, char* argv[]) {
    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem system;

    // ====================================================================================
    // Environment settings
    // ====================================================================================
    make_environment(system);

    // ====================================================================================
    // Barge
    // ====================================================================================
    auto barge = make_barge(system);

    // ====================================================================================
    // Crane model
    // ====================================================================================
    auto base_crane = std::make_shared<FrBody>();
    auto arm_crane = std::make_shared<FrBody>();
    make_crane(system, base_crane, arm_crane);

    // ====================================================================================
    // Motors
    // ====================================================================================
    make_motors_crane(system,barge,base_crane,arm_crane);

    // ====================================================================================
    // Hub model
    // ====================================================================================
    auto hub_box = std::make_shared<FrBody>();
    make_hub(system,hub_box,arm_crane);

    // ====================================================================================
    // Export Cable
    // ====================================================================================
    make_export_cable(system,hub_box);

    // ====================================================================================
    // Mooring system
    // ====================================================================================
    make_mooring(system,barge);

    // ====================================================================================
    // Run Simulation
    // ====================================================================================
    run_simulation(system);


}
