//
// Created by camille on 05/07/18.
//

#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include <chrono/physics/ChLinkMotorRotationTorque.h>
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_irrlicht/ChIrrCamera.h"

#include "frydom/frydom.h"


using namespace chrono;
using namespace frydom;


int main(int argc, char* argv[]) {

    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem system;
    //ChSystemSMC system;

    // --------------------------------------------------
    // Solid model
    // --------------------------------------------------

    auto barge = std::make_shared<FrHydroBody>();
    barge->SetName("Barge");
    barge->SetHydroMesh("Barge2.obj", true);
    barge->SetPos(chrono::ChVector<double>(0., 0., -1.));
    //barge->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    system.AddBody(barge);
    //barge->SetBodyFixed(true);

    // TODO: faire en sorte de ne pas avoir a construire un ChVector !
    barge->SetInertiaXX(chrono::ChVector<double>(1.02e11 * 2, 1.34e11 * 2, 1.28e11 * 2)); // Attention, le *2 partout ici est pour emuler la masse ajoutee...
    barge->SetInertiaXY(chrono::ChVector<double>(-9.79e3 * 2, 4.73e3 * 2, 1.71e2 * 2));
    barge->SetMass(69.892e6 * 2); // TODO: Caler avec Camille
    // TODO: faire en sorte de ne pas avoir a construire un ChVector !
    barge->SetCOG(chrono::ChVector<double>(0, 0, 1.015)); // TODO: Caler avec Camille
    // ====================================================================================
    // Adding forces to the platform
    // ====================================================================================

    // Linear Hydrostatics
    // -------------------
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    hstForce->GetStiffnessMatrix()->SetDiagonal(1.29e7, 4.2e7, 4.35e9);
    hstForce->GetStiffnessMatrix()->SetNonDiagonal(-1.97e3, -3.04e3, -3.69e4);
    barge->AddForce(hstForce);  // Comment to remove



    auto base_crane = std::make_shared<FrBody>();
    base_crane->SetName("Base_crane");
    base_crane->SetVisuMesh("BaseCrane.obj");
    //base_crane->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    base_crane->SetPos(chrono::ChVector<double>(0., +7.5, 2.15));
    //base_crane->SetPos(chrono::ChVector<double>(1., -3., 7.5));
    //base_crane->SetPos(chrono::ChVector<double>(-7.5, 0., 3.));
    base_crane->SetMass(5e5);
    //base_crane->SetInertiaXX(chrono::ChVector<double>(0, 0, 1.e8));
    system.AddBody(base_crane);
    //base_crane->SetBodyFixed(true);

    auto arm_crane = std::make_shared<FrBody>();
    arm_crane->SetName("Tige_crane");
    arm_crane->SetVisuMesh("TigeCrane.obj");
    //tige_crane->SetCOG(chrono::ChVector<double>(0., 0., 0.));
    arm_crane->SetPos(chrono::ChVector<double>(0., +5.5, 3.5));
    arm_crane->SetRot(Q_from_AngAxis(-CH_C_PI_4, VECT_X));
    arm_crane->SetMass(1e5);
    system.AddBody(arm_crane);
    //arm_crane->SetBodyFixed(true);

    auto hub_box = std::make_shared<FrBody>();
    hub_box->SetName("HubBox");
    hub_box->SetVisuMesh("HubBox.obj");
    //hub_box->SetCOG(chrono::ChVector<double>(0., 0., 0.));
    hub_box->SetPos(chrono::ChVector<double>(0, -7.93503, 3.1));
    hub_box->SetMass(1e3);
    system.AddBody(hub_box);
    //hub_box->SetBodyFixed(true);
    fmt::print("Poids du hub : {}\n", hub_box->GetMass() * system.GetEnvironment()->GetGravityAcceleration());


    // ---------------------------------------------
    // Hub Line
    // ---------------------------------------------

    auto A3_tige = arm_crane->CreateNode(ChVector<double>(0., -19., 0.));
    auto A4_hub = hub_box->CreateNode(ChVector<double>(0., 0., 1.));

    auto Test = A3_tige->GetAbsPos();
    fmt::print("En haut de la grue : {}, {}, {}\n", Test.x(), Test.y(), Test.z());

    // Line properties
    double Lu = 12.835;
    auto u = chrono::ChVector<double>(0, 0, -1);
    double q = 616.538;
    double EA = 1.5708e9;
    double A = 0.05;
    double E = EA / A;
    double breakTensionAsset = 100000;

    /*auto Catenary = std::make_shared<FrCatenaryLine>(A3_tige, A4_hub, true, E, A, Lu, q, u);
    Catenary->Initialize();
    system.AddLink(Catenary);*/

    auto DynamicLine = std::make_shared<FrDynamicCable>();
    DynamicLine->SetStartingNode(A3_tige);
    DynamicLine->SetEndingNode(A4_hub);
    DynamicLine->SetCableLength(Lu);
    DynamicLine->SetNumberOfElements(10);
    DynamicLine->SetLinearDensity(30);
    DynamicLine->SetDiameter(A);
    DynamicLine->SetYoungModulus(EA);
    DynamicLine->SetRayleighDamping(0.01);
    DynamicLine->Initialize();
    system.Add(DynamicLine);


    // ---------------------------------------------
    // Mooring Lines
    // ---------------------------------------------



    auto ColorAsset = std::make_shared<ChColorAsset>(ChColor(1.f, 0.f, 0.0f));

    auto buoy1 = std::make_shared<FrMooringBuoy>(1.5,4e3,true,1e2);
    buoy1->SetName("Buoy1");
    buoy1->SetPos(ChVector<>(-5.,-27.5,0));
    buoy1->SetCOG(ChVector<>(0, 0, 0));
    //buoy1->SetBodyFixed(true);
    buoy1->AddAsset(ColorAsset);
    system.AddBody(buoy1);

    auto buoy2 = std::make_shared<FrMooringBuoy>(1.5,4e3,true,1e2);
    buoy2->SetName("Buoy2");
    buoy2->SetPos(ChVector<>(5.,-27.5,0));
    buoy2->SetCOG(ChVector<>(0, 0, 0));
    //buoy2->SetBodyFixed(true);
    buoy2->AddAsset(ColorAsset);
    system.AddBody(buoy2);

    auto buoy3 = std::make_shared<FrMooringBuoy>(1.5,4e3,true,1e2);
    buoy3->SetName("Buoy3");
    buoy3->SetPos(ChVector<>(-5.,27.5,0));
    buoy3->SetCOG(ChVector<>(0, 0, 0));
    //buoy3->SetBodyFixed(true);
    buoy3->AddAsset(ColorAsset);
    system.AddBody(buoy3);

    auto buoy4 = std::make_shared<FrMooringBuoy>(1.5,4e3,true,1e2);
    buoy4->SetName("Buoy4");
    buoy4->SetPos(ChVector<>(5.,27.5,0));
    buoy4->SetCOG(ChVector<>(0, 0, 0));
    //buoy4->SetBodyFixed(true);
    buoy4->AddAsset(ColorAsset);
    system.AddBody(buoy4);

    /// Mooring lines length
    Lu = 110;
    double Lv = 15.5;

    auto B1 = barge->CreateNode(ChVector<double>(-5, -12.5, 0.));
    auto B2 = barge->CreateNode(ChVector<double>(5, -12.5, 0.));
    auto B3 = barge->CreateNode(ChVector<double>(-5, 12.5, 0.));
    auto B4 = barge->CreateNode(ChVector<double>(5, 12.5, 0.));

    auto BB1 = buoy1->CreateNode(ChVector<double>(0,0,0));
    auto BB2 = buoy2->CreateNode(ChVector<double>(0,0,0));
    auto BB3 = buoy3->CreateNode(ChVector<double>(0,0,0));
    auto BB4 = buoy4->CreateNode(ChVector<double>(0,0,0));

    auto WB1 = system.GetWorldBody()->CreateNode(ChVector<double>(-25, -125, -30.));
    auto WB2 = system.GetWorldBody()->CreateNode(ChVector<double>(25, -125, -30.));
    auto WB3 = system.GetWorldBody()->CreateNode(ChVector<double>(-25, 125, -30.));
    auto WB4 = system.GetWorldBody()->CreateNode(ChVector<double>(25, 125, -30.));

    auto MooringLine1 = std::make_shared<FrCatenaryLine>(B1, BB1, true, E, A, Lv, q, u);
    auto MooringLine2 = std::make_shared<FrCatenaryLine>(B2, BB2, true, E, A, Lv, q, u);
    auto MooringLine3 = std::make_shared<FrCatenaryLine>(B3, BB3, true, E, A, Lv, q, u);
    auto MooringLine4 = std::make_shared<FrCatenaryLine>(B4, BB4, true, E, A, Lv, q, u);

    auto MooringLine1b = std::make_shared<FrCatenaryLine>(BB1, WB1, true, E, A, Lu, q, u);
    auto MooringLine2b = std::make_shared<FrCatenaryLine>(BB2, WB2, true, E, A, Lu, q, u);
    auto MooringLine3b = std::make_shared<FrCatenaryLine>(BB3, WB3, true, E, A, Lu, q, u);
    auto MooringLine4b = std::make_shared<FrCatenaryLine>(BB4, WB4, true, E, A, Lu, q, u);

    MooringLine1->SetBreakingTension(breakTensionAsset);
    MooringLine2->SetBreakingTension(breakTensionAsset);
    MooringLine3->SetBreakingTension(breakTensionAsset);
    MooringLine4->SetBreakingTension(breakTensionAsset);
    MooringLine1b->SetBreakingTension(breakTensionAsset);
    MooringLine2b->SetBreakingTension(breakTensionAsset);
    MooringLine3b->SetBreakingTension(breakTensionAsset);
    MooringLine4b->SetBreakingTension(breakTensionAsset);

    MooringLine1->Initialize();
    MooringLine2->Initialize();
    MooringLine3->Initialize();
    MooringLine4->Initialize();
    MooringLine1b->Initialize();
    MooringLine2b->Initialize();
    MooringLine3b->Initialize();
    MooringLine4b->Initialize();

    system.AddLink(MooringLine1);
    system.AddLink(MooringLine2);
    system.AddLink(MooringLine3);
    system.AddLink(MooringLine4);
    system.AddLink(MooringLine1b);
    system.AddLink(MooringLine2b);
    system.AddLink(MooringLine3b);
    system.AddLink(MooringLine4b);

    // ----------------------------------------------
    // Motors
    // ----------------------------------------------
    // Barge - Crane base:
    auto rotmotor_crane = std::make_shared<ChLinkMotorRotationSpeed>();
    rotmotor_crane->Initialize(base_crane, barge, ChFrame<>(ChVector<>(0., 7.5, 2.)));
    system.Add(rotmotor_crane);
    auto rw_crane = std::make_shared<ChFunction_Const>(0.2);
    rotmotor_crane->SetSpeedFunction(rw_crane);

    // Crane base - Crane arm
    auto rotmotor_arm = std::make_shared<ChLinkMotorRotationAngle>();
    rotmotor_arm->Initialize(arm_crane, base_crane, ChFrame<>(ChVector<>(0., 5.5, 3.5), CH_C_PI_2, VECT_Y));
    system.Add(rotmotor_arm);
    auto rw_arm = std::make_shared<ChFunction_Const>(0.);
    rotmotor_arm->SetAngleFunction(rw_arm);

    // --------------------------------------------------
    // Simulation Parameter
    // --------------------------------------------------
    // TODO : les lignes suivantes jusqu'a app devraient etre gerees par defaut suivant qu'on a un cable ou pas ...
    // TODO: voir si on peut specifier ces reglages pour un modele dans cable
    // Si NON, Peut-on avoir un reglage auto du solveur MINRES
    system.SetSolverType(
            chrono::ChSolver::Type::MINRES);  // TODO: voir si on peut regler ce solveur pour des simulations sans cable
    system.SetSolverWarmStarting(true);
    system.SetMaxItersSolverSpeed(
            1000);  // TODO: mettre en place une adaptation lorsqu'on a un residu du solveur trop important
    system.SetMaxItersSolverStab(200);
    system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<chrono::ChSolverMINRES>(system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    system.SetupInitial();

    // -----------------------------------------------
    // Simulation
    // -----------------------------------------------

    double dt = 0.00125;

    //system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);
    system.Initialize();

    auto app = FrIrrApp(system);
    auto myCam = app.AddCustomCamera(irr::core::vector3df(80, 0, 0), irr::core::vector3df(0, 0, 0));
    //app.SetShowInfos(true);
    app.SetVideoframeSave(false);
    app.Run();

}