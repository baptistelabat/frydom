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

#include <chrono/physics/ChLinkMotorLinearPosition.h>
#include "frydom/frydom.h"


using namespace chrono;
using namespace frydom;

int main() {
    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem my_system;
    my_system.GetEnvironment()->GetSeabed()->SetDepth(-100);

    // --------------------------------------------------
    // Bodies
    // --------------------------------------------------

    std::shared_ptr<FrBody> movingBody;
    movingBody = std::make_shared<FrSphere>(1, 1000, true);
    movingBody->SetName("pendulum");
    movingBody->SetPos(ChVector<>(0., 0, 10));
    //movingBody->SetBodyFixed(true);
    my_system.Add(movingBody);

    // --------------------------------------------------
    // FrNodes
    // --------------------------------------------------
    // TODO: Ajouter une méthode CreateNode avec un ChFrame en entrée
    auto Node1 = my_system.GetWorldBody()->CreateNode(ChVector<>(10,0,10));
    //Node1->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    auto movingNode = movingBody->CreateNode(ChVector<>(0,0,0));
    //movingNode->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));

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

    auto Catenary1 = std::make_shared<FrCatenaryLine>(Node1, movingNode, true, E, A, Lu, q, u);
    Catenary1->Initialize();
    my_system.AddLink(Catenary1);


    auto my_Buoy = std::make_shared<FrAssetBuoy>(ChVector<>(10,0,0), 0.1, ChColor(1.f,0.f,0.f));
    Catenary1->AddAssetComponent(my_Buoy);

    Catenary1->AddAssetComponent(std::make_shared<FrAssetClumpWeight>());



    // --------------------------------------------------
    // Linear Motor
    // --------------------------------------------------
    // Create the linear motor
    auto motor = std::make_shared<ChLinkMotorLinearPosition>();
    //motor->Initialize(movingBody,my_system.GetWorldBody(),true,ChFrame<>(ChVector<>(0,0,0)),ChFrame<>(ChVector<>(0,0,10)));
    motor->Initialize(movingBody,              // body A (slave)
                      my_system.GetWorldBody(),               // body B (master)
                      ChFrame<>(ChVector<>(0,0,10))  // motor frame, in abs. coords
    );
    my_system.Add(motor);
    // Create a ChFunction to be used for the ChLinkMotorLinearPosition
    auto msp = std::make_shared<ChFunction_Sine>(CH_C_PI_2,0.5,1);
    // Let the motor use this motion function:
    motor->SetMotionFunction(msp);

    // --------------------------------------------------
    // Simulation Parameter
    // --------------------------------------------------
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

    double dt = 0.01;
    my_system.SetStep(dt);
    my_system.Initialize();

    auto app = FrIrrApp(my_system);
    app.AssetBindAll();
    app.AssetUpdateAll();
    app.AddTypicalCamera(irr::core::vector3df(1, 20, 200), irr::core::vector3df(0, 0, 10));
    app.SetVideoframeSave(true);
    app.Run();


}