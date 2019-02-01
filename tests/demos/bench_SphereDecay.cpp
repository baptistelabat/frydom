//
// Created by camille on 01/02/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    std::cout << " ===================================================== \n"
                 " Benchmark test : Sphere Decay \n"
                 " ===================================================== " << std::endl;

    // -- System

    FrOffshoreSystem_ system;

    // -- Body

    auto body = system.NewBody();

    Position COGPosition(0., 0., -2.);
    FrFrame_ COGFrame(COGPosition, FrRotation_(), NWU);

    body->SetPosition(Position(0., 0., 0.), NWU);

    // -- Inertia

    double mass = 2.618E5;

    double Ixx  = 1.690E6;
    double Iyy  = 1.690E6;
    double Izz  = 2.606E6;

    FrInertiaTensor_ InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGFrame, NWU);

    body->SetInertiaTensor(InertiaTensor);

    // -- Hydrodynamics

    auto hdb = std::make_shared<FrHydroDB_>("sphere_hdb.h5");

    auto eqFrame = std::make_shared<FrEquilibriumFrame_>(body.get());
    system.AddPhysicsItem(eqFrame);

    hdb->Map(0, body.get(), eqFrame);

    // -- Hydrostatic

    auto forceHst = std::make_shared<FrLinearHydrostaticForce_>(hdb.get());
    body->AddExternalForce(forceHst);

    // -- Radiation

    auto radiationModel = std::make_shared<FrRadiationConvolutionModel_>(hdb);
    system.AddPhysicsItem(radiationModel);

    auto radiationForce = std::make_shared<FrRadiationConvolutionForce_>(radiationModel);

    body->AddExternalForce(radiationForce);

    // -- Simulation

    auto dt = 0.005;

    system.SetTimeStep(dt);
    system.Initialize();

    body->SetPosition(Position(0., 0., 3.), NWU);

    auto time = 0.;

    std::ofstream myfile;
    myfile.open("sphere_position_allforce_new.csv");
    myfile << "time;X;Y;Z" << std::endl;

    while (time < 20.) {

        time += dt;
        system.AdvanceTo(time);

        std::cout << "time : " << time << " ; position of the body = "
                  << body->GetPosition(NWU).GetX() << " ; "
                  << body->GetPosition(NWU).GetY() << " ; "
                  << body->GetPosition(NWU).GetZ()
                  << std::endl;

        myfile << time << ";" << body->GetPosition(NWU).GetX() << ";"
                              << body->GetPosition(NWU).GetY() << ";"
                              << body->GetPosition(NWU).GetZ() << std::endl;
    }

    //myfile.close();

    std::cout << "============================== End ===================================== " << std::endl;

} // end namespace frydom