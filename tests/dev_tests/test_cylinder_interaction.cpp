//
// Created by camille on 10/04/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    std::cout << " ========================== Test Cylinder interaction ================== " << std::endl;

    // System

    FrOffshoreSystem system;

    // Environment

    auto ocean = system.GetEnvironment()->GetOcean();
    auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWaveHeight(0.001);
    waveField->SetWavePeriod(M_PI/4.);
    waveField->SetDirection(0., DEG, NWU, GOTO);

    // Bodies

    auto cyl1 = system.NewBody();
    cyl1->SetPosition(Position(0., -0.4, 0.), NWU);

    //auto cyl2 = system.NewBody();
    //cyl2->SetPosition(Position(0., +0.4, 0.), NWU);

    double mass = 12.88;
    FrFrame COGFrame(Position(0., 0., 0.), FrRotation(), NWU);
    FrInertiaTensor InertiaTensor(mass, 1., 1., 1.,  0., 0., 0., COGFrame, NWU);

    cyl1->SetInertiaTensor(InertiaTensor);
    //cyl2->SetInertiaTensor(InertiaTensor);

    // DOFMask

    cyl1->GetDOFMask()->MakeItLocked();
    cyl1->GetDOFMask()->SetLock_Z(false);

    //cyl2->GetDOFMask()->MakeItLocked();
    //cyl2->GetDOFMask()->SetLock_Z(false);

    // Hydrodynamic

    auto hdb = make_hydrodynamic_database("CylinderInteraction.hdb5");

    auto eqFrame1 = std::make_shared<FrEquilibriumFrame>(cyl1.get());
    //auto eqFrame2 = std::make_shared<FrEquilibriumFrame>(cyl2.get());

    hdb->Map(0, cyl1.get(), eqFrame1);
    //hdb->Map(1, cyl2.get(), eqFrame2);

    auto radiationModel = make_radiation_convolution_model(hdb, &system);

    // Hydrostatic

    auto forceHst1 = make_linear_hydrostatic_force(hdb, cyl1);
    //auto forceHst2 = make_linear_hydrostatic_force(hdb, cyl2);

    // Excitation

    auto excitation1 = make_linear_excitation_force(hdb, cyl1);
    //auto excitation2 = make_linear_excitation_force(hdb, cyl2);

    // Simulation

    auto dt = 0.005;

    system.SetTimeStep(dt);

    std::cout << "Initialize ..." << std::endl;

    system.Initialize();

    std::cout << "Run simulation ..." << std::endl;

    double time = 0.;

    while (time < 50.) {

        time += dt;

        system.AdvanceTo(time);

        std::cout << "Time : " << time << " s" << std::endl;

        std::cout << "Position cyl1 : " << cyl1->GetPosition(NWU).GetX() << ";"
                                        << cyl1->GetPosition(NWU).GetY() << ";"
                                        << cyl1->GetPosition(NWU).GetZ() << std::endl;

        //std::cout << "Position cyl2 : " << cyl2->GetPosition(NWU).GetX() << ";"
        //                                << cyl2->GetPosition(NWU).GetY() << ";"
        //                                << cyl2->GetPosition(NWU).GetZ() << std::endl;



    }

    std::cout << "======================================= End ============================= " << std::endl;


}