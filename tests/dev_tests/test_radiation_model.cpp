//
// Created by camille on 23/01/19.
//

#include "frydom/frydom.h"


using namespace frydom;

int main(int argc, char* argv[]) {

    std::cout << "=============================== Test radiation model =================== " << std::endl;

    // ---- System

    FrOffshoreSystem_ system;

    system.GetEnvironment()->GetOcean()->SetInfiniteDepth();

    auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

    //auto waveField = freeSurface->SetAiryIrregularWaveField();
    //auto Jonswap = waveField->SetJonswapWaveSpectrum(0.1, 9.);
    //waveField->SetWaveFrequencies(0.5, 2., 40.);
    //waveField->SetMeanWaveDirection(SOUTH(NWU), NWU, GOTO);

    auto waveField = freeSurface->SetAiryRegularWaveField();
    waveField->SetWavePeriod(9.);
    waveField->SetWaveHeight(0.);
    waveField->SetDirection(SOUTH(NWU), NWU, GOTO);

    auto timeRamp = system.GetEnvironment()->GetTimeRamp();
    timeRamp->Deactivate();

    // --- Body

    auto body = system.NewBody();

    Position COGPos(0.22, 0.22, 2.92);
    FrFrame_ COGFrame(COGPos, FrRotation_(), NWU);

    body->SetPosition(Position(0., 0., 0.), NWU);

    // - Inertia

    double mass = 3.22114e7;

    double Ixx               = 2.4e11;
    double Iyy               = 2.3e11;
    double Izz               = 2e12;

    FrInertiaTensor_ InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGFrame, NWU);

    body->SetInertiaTensor(InertiaTensor);

    // --- Hydrodynamic

    auto hdb = std::make_shared<FrHydroDB_>("Platform_HDB.hdb5");

    auto eqFrame = std::make_shared<FrEquilibriumFrame_>(body.get());
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