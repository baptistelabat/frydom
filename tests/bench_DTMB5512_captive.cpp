//
// Created by camille on 12/07/18.
//

#include "frydom/frydom.h"
#include "frydom/frydom_dice.h"

using namespace frydom;
using namespace chrono;

// ----------------------------------------------------------
// Ship model : DTMB5512 (IIHR - Yoon 2009)
// ----------------------------------------------------------

std::shared_ptr<DShip> DTMB5512(FrOffshoreSystem* system) {

    auto ship_pos = ChVector<>(0., 0., 0.);

    auto ship = std::make_shared<DShip>();
    system->AddBody(ship);

    // Geometry properties
    ship->SetName("DTMB5512");
    ship->SetHydroMesh("DTMB5512.obj", true);
    ship->SetLpp(3.048);
    ship->SetMass(86.0);
    ship->SetCOG(ChVector<>(0., 0., -0.036));
    ship->SetInertiaXX(ChVector<>(1.98, 53.88, 49.99));
    ship->SetPos(ChVector<>(0., 0., 0.));
    ship->SetEquilibriumFrame(MeanMotion, 60.);

    // Hydrostatics
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    auto hstStiffness = hstForce->GetStiffnessMatrix();
    hstStiffness->SetDiagonal(9.68e3, 8.97e1, 5.48e3);
    ship->AddForce(hstForce);

    // Hydrodynamics
    system->SetHydroDB("DTMB5512_hdb.h5");
    auto hydroMapIndex = system->GetHydroMapNb()-1;
    system->GetHydroMapper(hydroMapIndex)->Map(ship, 0);

    // Radiation model
    auto radModel = std::make_shared<FrRadiationConvolutionModel>(system->GetHydroDB(hydroMapIndex), system);
    radModel->SetHydroMapIndex(hydroMapIndex); // TODO : patch hydro map multibody
    radModel->AddRadiationForceToHydroBody(ship);

    // Wave Probe
    auto waveField = system->GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    //auto waveProbe = waveField->NewWaveProbe(ship_pos.x(), ship_pos.y());
    auto waveProbe = std::make_shared<FrLinearWaveProbe>();
    waveProbe->AttachedNode(ship->GetEquilibriumFrame());
    waveField->AddWaveProbe(waveProbe);
    waveProbe->Initialize();

    // Wave Excitation force
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    ship->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);
    excForce->SetHydroMapIndex(hydroMapIndex);

    return ship;
}


int main(int argc, char* argv[]) {

    // ------------------------------------------------------
    // System
    // ------------------------------------------------------

    FrOffshoreSystem system;

    // ------------------------------------------------------
    // Environment
    // ------------------------------------------------------

    system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_REGULAR);
    auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    waveField->SetRegularWaveHeight(0.2);
    waveField->SetRegularWavePeriod(0.988);
    waveField->GetSteadyElevation(0, 0);

    // ------------------------------------------------------
    // Body
    // ------------------------------------------------------

    auto ship = DTMB5512(&system);

    auto vspeed = ChVector<>(1.04, 0., 0.);
    ship->SetPos_dt(vspeed);
    ship->SetSteadyVelocity(vspeed);

    // ------------------------------------------------------
    // Simulation
    // ------------------------------------------------------

    double dt = 0.01;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);
    system.Initialize();

    auto app = FrIrrApp(system);
    app.AddTypicalCamera(irr::core::vector3df(0, 0, 10), irr::core::vector3df(0, 0, -1));

    app.Run();

// ==========================================================
}

