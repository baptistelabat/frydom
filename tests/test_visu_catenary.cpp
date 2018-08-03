//
// Created by Lucas Letournel on 18/07/18.
//

#include <chrono/physics/ChLinkMotorLinearPosition.h>
// #include <chrono/physics/ChSystemNSC.h>
#include "frydom/frydom.h"
#include <fmt/format.h>

//#include <chrono/physics/ChLinkLock.h>

using namespace chrono;
using namespace frydom;

int main() {
    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem my_system;

    // --------------------------------------------------
    // Bodies
    // --------------------------------------------------

    std::shared_ptr<FrBody> base;
    base = std::make_shared<FrSphere>(1, 100, true);
    base->SetName("base");
    base->SetPos(ChVector<>(-10, 0, 10));
    base->SetBodyFixed(true);
    my_system.Add(base);

    std::shared_ptr<FrBody> pendulum1;
    pendulum1 = std::make_shared<FrSphere>(1, 1000, true);
    pendulum1->SetName("pendulum");
    pendulum1->SetPos(ChVector<>(0., 0, 10));
    //pendulum1->SetBodyFixed(true);
    my_system.Add(pendulum1);

    // --------------------------------------------------
    // FrNodes
    // --------------------------------------------------
    // TODO: Ajouter une méthode CreateNode avec un ChFrame en entrée
    //auto Node1 = my_system.GetWorldBody()->CreateNode(ChVector<>(-10,0,10));
    //auto Node2 = my_system.GetWorldBody()->CreateNode(ChVector<>(10,0,10));
    auto baseNode = base->CreateNode(ChVector<>(0,0,0));
    //baseNode->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    auto pendulumNode = pendulum1->CreateNode(ChVector<>(0,0,0));
    //pendulumNode->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));

    // --------------------------------------------------
    // Catenary Line
    // --------------------------------------------------
    // Line properties
    double Lu = 12.;
    auto u = chrono::ChVector<double>(0, 0, -1);
    double q = 616.538;
    double EA = 1.5708e9;
    double A = 0.05;
    double E = EA/A;

    auto Catenary1 = std::make_shared<FrCatenaryLine>(baseNode, pendulumNode, true, E, A, Lu, q, u);
    Catenary1->Initialize();
    my_system.AddLink(Catenary1);

/*    auto Catenary2 = std::make_shared<FrCatenaryLine>(pendulumNode, Node2, true, E, A, Lu, q, u);
    Catenary2->Initialize();
    my_system.AddLink(Catenary2);*/

/*    // --------------------------------------------------
    // Linear link
    // --------------------------------------------------
    auto linear_link = std::make_shared<ChLinkLockPointLine>();
    linear_link->Initialize(pendulum1,my_system.GetWorldBody(),ChCoordsys<>(ChVector<>(0,0,10),CH_C_PI_2,ChVector<>(0,1,0)));
    my_system.AddLink(linear_link);*/


    // --------------------------------------------------
    // Linear Motor
    // --------------------------------------------------
    // Create the linear motor
    auto motor = std::make_shared<ChLinkMotorLinearPosition>();
    //motor->Initialize(pendulum1,my_system.GetWorldBody(),true,ChFrame<>(ChVector<>(0,0,0)),ChFrame<>(ChVector<>(0,0,10)));
    motor->Initialize(pendulum1,              // body A (slave)
                      my_system.GetWorldBody(),               // body B (master)
                      ChFrame<>(ChVector<>(0,0,10))  // motor frame, in abs. coords
    );
    my_system.Add(motor);
    // Create a ChFunction to be used for the ChLinkMotorLinearPosition
    auto msp = std::make_shared<ChFunction_Sine>(CH_C_PI_2,0.5,1);
    // Let the motor use this motion function:
    motor->SetMotionFunction(msp);

/*    //--------------------------------------------------
    // Test
    //--------------------------------------------------
    auto LinkDist = std::make_shared<ChLinkDistance>();
    LinkDist->Initialize(base,pendulum1,true,ChVector<>(0,0,0),ChVector<>(0,0,0),false,10);
    LinkDist->AddAsset(std::make_shared<ChPointPointSegment>());
    my_system.AddLink(LinkDist);
*/

/*
    // --------------------------------------------------
    // Dynamic Line
    // --------------------------------------------------
    // Body
    double radius = 1;
    double mass = 100;
    auto sphere = std::make_shared<FrSphere>(radius, mass); // TODO: permettre de donner une masse

    sphere->SetPos(chrono::ChVector<>(10, -10, 20));
    sphere->SetName("sphere");
    my_system.AddBody(sphere);


    auto cable = std::make_shared<FrDynamicCable>();
    cable->SetCableLength(10);  // TODO: augmenter la taille et la discretisation une fois le cable attache...
    cable->SetNumberOfElements(20);
    cable->SetLinearDensity(30);
    cable->SetDiameter(0.05);
    cable->SetYoungModulus(1e9);
    cable->SetRayleighDamping(0.01);

    auto anchor = my_system.GetWorldBody()->CreateNode(chrono::ChVector<>(0, -10, 20));
    cable->SetStartingNode(anchor);

    auto node = sphere->CreateNode();
    cable->SetEndingNode(node);

    cable->Initialize();
    my_system.Add(cable);
*/

    // TODO : les lignes suivantes jusqu'a app devraient etre gerees par defaut suivant qu'on a un cable ou pas ...
    // TODO: voir si on peut specifier ces reglages pour un modele dans cable
    // Si NON, Peut-on avoir un reglage auto du solveur MINRES
    my_system.SetSolverType(chrono::ChSolver::Type::MINRES);  // TODO: voir si on peut regler ce solveur pour des simulations sans cable
    my_system.SetSolverWarmStarting(true);
    my_system.SetMaxItersSolverSpeed(1000);  // TODO: mettre en place une adaptation lorsqu'on a un residu du solveur trop important
    my_system.SetMaxItersSolverStab(200);
    my_system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<chrono::ChSolverMINRES>(my_system.GetSolver());
//    msolver->SetVerbose(true);
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

//    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    my_system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    my_system.SetupInitial();



/*
    // -----------------------------------------------
    // Simulation
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
    }
*/

    double dt = 0.01;

    //my_system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    my_system.SetStep(dt);
    my_system.Initialize();

    auto app = FrIrrApp(my_system);
    app.AssetBindAll();
    app.AssetUpdateAll();
    app.AddTypicalCamera(irr::core::vector3df(1, 20, 10), irr::core::vector3df(0, 0, 10));
    app.Run();


}