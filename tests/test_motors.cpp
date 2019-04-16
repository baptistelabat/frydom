//
// Created by Lucas Letournel on 20/07/18.
//

#include "frydom/frydom.h"
#include <chrono/physics/ChLinkMotorRotationSpeed.h>

using namespace chrono;
using namespace frydom;

using namespace chrono::irrlicht;
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

int main() {
    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem my_system;
    //ChSystemSMC my_system;

    // --------------------------------------------------
    // Bodies
    // --------------------------------------------------

    std::shared_ptr<FrBody> base;
    base = std::make_shared<FrSphere>(0.1, 1000, true);
    base->SetName("base");
    base->SetPos(ChVector<>(0, 0, 10));
    base->SetBodyFixed(true);
    my_system.Add(base);

    std::shared_ptr<FrBody> pendulum1;
    pendulum1 = std::make_shared<FrSphere>(1, 1000, true);
    pendulum1->SetName("pendulum");
    pendulum1->SetPos(ChVector<>(1.2, 0, 10));
    my_system.Add(pendulum1);

    std::shared_ptr<FrBody> ref;
    ref = std::make_shared<FrSphere>(0.1, 1000, true);
    ref->SetName("ref");
    ref->SetPos(ChVector<>(0, 3, 10));
    ref->SetBodyFixed(true);
    my_system.Add(ref);

    // --------------------------------------------------
    // FrNodes
    // --------------------------------------------------
    // TODO: Ajouter une méthode CreateNode avec un ChFrame en entrée
    //auto baseNode = base->CreateNode(ChVector<>(0, 0, 0));
    //baseNode->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    //auto pendulumNode = pendulum1->CreateNode(ChVector<>(-1.2, 0, 0));
    //pendulumNode->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));

    // --------------------------------------------------
    // Motors
    // --------------------------------------------------
    // Create the motor
    auto rotmotor1 = std::make_shared<chrono::ChLinkMotorRotationSpeed>();

    // Connect the rotor and the stator and add the motor to the system: all different methods available
    //rotmotor1->Initialize(pendulum1, base, ChFrame<>(ChVector<>(0, 0, 10)));
    //rotmotor1->Initialize(pendulum1, base, true, ChFrame<>(ChVector<>(-1.2, 0, 0)), ChFrame<>(ChVector<>(0, 0, 0)));
    //rotmotor1->Initialize(pendulum1, base, false, ChFrame<>(ChVector<>(0, 0, 10)), ChFrame<>(ChVector<>(0, 0, 10)));
    rotmotor1->Initialize(pendulum1, base, true, ChVector<>(-1.2, 0, 0), ChVector<>(0, 0, 0),ChVector<>(1, 0, 0), ChVector<>(1, 0, 0));
    //rotmotor1->Initialize(pendulum1, base, false, ChVector<>(0, 0, 10), ChVector<>(0, 0, 10),ChVector<>(1, 0, 0), ChVector<>(1, 0, 0));// Not adapted to this case?
    my_system.Add(rotmotor1);

    // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
    auto mwspeed = std::make_shared<ChFunction_Const>(CH_C_PI_2); // constant angular speed, in [rad/s], 1PI/s =180�/s
    // Let the motor use this motion function:
    rotmotor1->SetSpeedFunction(mwspeed);

    my_system.Update(true);

/*
    // -----------------------------------------------
    // Quick Simulation
    // -----------------------------------------------

    my_system.Update(true);
    double chronoTime = 0;
    while (chronoTime < 5) {
        chronoTime += 0.01;
        // PERFORM SIMULATION UP TO chronoTime
        my_system.DoFrameDynamics(chronoTime);
        // Print something on the console..
        GetLog() << "Time: " << chronoTime
                 << "  x_Pendulum: " << pendulum1->GetPos().x() << "\n";
    }*/


    // -----------------------------------------------
    // Irrlicht with FrOffshoreSystem
    // -----------------------------------------------

    double dt = 0.05;

    my_system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    my_system.SetStep(dt);
    //my_system.Initialize();

    auto app = FrIrrApp(my_system);
    app.AddTypicalCamera(irr::core::vector3df(1, 1, 15), irr::core::vector3df(0, 0, 10));

    app.Run();
}