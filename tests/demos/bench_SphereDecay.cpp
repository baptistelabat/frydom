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

    body->GetDOFMask()->SetLock_X(true);
    body->GetDOFMask()->SetLock_Y(true);
    body->GetDOFMask()->SetLock_Rx(true);
    body->GetDOFMask()->SetLock_Ry(true);
    body->GetDOFMask()->SetLock_Rz(true);

    // -- Inertia

    double mass = 2.618E5;

    double Ixx  = 1.690E6;
    double Iyy  = 1.690E6;
    double Izz  = 2.606E6;

    FrInertiaTensor_ InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGFrame, NWU);

    body->SetInertiaTensor(InertiaTensor);

    // -- Hydrodynamics

    auto hdb = make_hydrodynamic_database("sphere_hdb.h5");

    auto eqFrame = std::make_shared<FrEquilibriumFrame_>(body.get());
    system.AddPhysicsItem(eqFrame);

    hdb->Map(0, body.get(), eqFrame);

    // -- Hydrostatic

    auto forceHst = make_linear_hydrostatic_force(hdb, body);

    // -- Radiation

    auto radiationModel = make_radiation_convolution_model(hdb, &system);

    radiationModel->SetImpulseResponseSize(body.get(), 6., 0.1);

    // -- Simulation

    auto dt = 0.005;

    system.SetTimeStep(dt);
    system.Initialize();

    body->SetPosition(Position(0., 0., 3.), NWU);

    auto time = 0.;

    // ##CC
    std::ofstream myfile;
    myfile.open("sphere_position.csv");
    myfile << "time;X;Y;Z" << std::endl;
    // ##CC

    while (time < 40.) {

        time += dt;
        system.AdvanceTo(time);

        // ##CC
        std::cout << "time : " << time << " ; position of the body = "
                  << body->GetPosition(NWU).GetX() << " ; "
                  << body->GetPosition(NWU).GetY() << " ; "
                  << body->GetPosition(NWU).GetZ()
                  << std::endl;

        myfile << time << ";" << body->GetPosition(NWU).GetX() << ";"
                              << body->GetPosition(NWU).GetY() << ";"
                              << body->GetPosition(NWU).GetZ() << std::endl;
        // ##CC
    }

    myfile.close();

    std::cout << "============================== End ===================================== " << std::endl;

} // end namespace frydom