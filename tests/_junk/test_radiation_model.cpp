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


using namespace frydom;

int main(int argc, char* argv[]) {

    std::cout << "=============================== Test radiation model =================== " << std::endl;

    cppfs::FilePath resources_path(std::string(RESOURCES_PATH));

    // ---- System

    FrOffshoreSystem system;

    system.GetEnvironment()->GetOcean()->SetInfiniteDepth();

    auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

    auto waveField = freeSurface->SetAiryRegularWaveField();
    waveField->SetWavePeriod(9.);
    waveField->SetWaveHeight(0.);
    waveField->SetDirection(SOUTH(NWU), NWU, GOTO);

    auto timeRamp = system.GetEnvironment()->GetTimeRamp();
    timeRamp->SetActive(false);

    // --- Body

    auto body = system.NewBody();

    Position COGPos(0.22, 0.22, 2.92);

    body->SetPosition(Position(0., 0., 0.), NWU);

    // - Inertia

    double mass = 3.22114e7;

    double Ixx               = 2.4e11;
    double Iyy               = 2.3e11;
    double Izz               = 2e12;

    FrInertiaTensor InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGPos, NWU);

    body->SetInertiaTensor(InertiaTensor);

    // --- Hydrodynamic

    auto hdb = std::make_shared<FrHydroDB>(resources_path.resolve("Platform_HDB.hdb5").path());

    auto eqFrame = std::make_shared<FrEquilibriumFrame>(body.get());
    system.AddPhysicsItem(eqFrame);

    hdb->Map(0, body.get(), eqFrame);

    // - Hydrostatic

    auto forceHst = make_linear_hydrostatic_force(hdb, body);

    // - Excitation

    auto excitationForce = make_linear_excitation_force(hdb, body);

    // - Radiation

    auto radiationModel = make_radiation_convolution_model(hdb, &system);

    // --- Simulation

    auto dt = 0.01;

    system.SetTimeStep(dt);

    system.Initialize();

    body->SetPosition(Position(0., 0., 1.), NWU);

    auto time = 0.;

    while(time < 10.) {
        time += dt;
        system.AdvanceTo(time);

        std::cout << "time : " << time << " ; position of the body = "
                  << body->GetPosition(NWU).GetX() << " ; "
                  << body->GetPosition(NWU).GetY() << " ; "
                  << body->GetPosition(NWU).GetZ()
                  << std::endl;
    }

    std::cout << "============================== End ===================================== " << std::endl;
}