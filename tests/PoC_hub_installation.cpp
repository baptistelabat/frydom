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
    // Run Simulation
    // ====================================================================================
    run_simulation(system);


}
