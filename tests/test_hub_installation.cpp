//
// Created by camille on 05/07/18.
//

#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include <chrono/physics/ChLinkMotorRotationTorque.h>
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

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
    double breakTensionAsset = 0;//5000000;

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

    /// Mooring lines length
    Lu = 120;

    auto B1 = barge->CreateNode(ChVector<double>(-2.5, -12.5, 0.));
    auto B2 = barge->CreateNode(ChVector<double>(2.5, -12.5, 0.));
    auto B3 = barge->CreateNode(ChVector<double>(-2.5, 12.5, 0.));
    auto B4 = barge->CreateNode(ChVector<double>(2.5, 12.5, 0.));

    auto WB1 = system.GetWorldBody()->CreateNode(ChVector<double>(-25, -125, -30.));
    auto WB2 = system.GetWorldBody()->CreateNode(ChVector<double>(25, -125, -30.));
    auto WB3 = system.GetWorldBody()->CreateNode(ChVector<double>(-25, 125, -30.));
    auto WB4 = system.GetWorldBody()->CreateNode(ChVector<double>(25, 125, -30.));

    auto MooringLine1 = std::make_shared<FrCatenaryLine>(B1, WB1, true, E, A, Lu, q, u);
    auto MooringLine2 = std::make_shared<FrCatenaryLine>(B2, WB2, true, E, A, Lu, q, u);
    auto MooringLine3 = std::make_shared<FrCatenaryLine>(B3, WB3, true, E, A, Lu, q, u);
    auto MooringLine4 = std::make_shared<FrCatenaryLine>(B4, WB4, true, E, A, Lu, q, u);

    MooringLine1->SetBreakingTension(breakTensionAsset);
    MooringLine2->SetBreakingTension(breakTensionAsset);
    MooringLine3->SetBreakingTension(breakTensionAsset);
    MooringLine4->SetBreakingTension(breakTensionAsset);

    MooringLine1->Initialize();
    MooringLine2->Initialize();
    MooringLine3->Initialize();
    MooringLine4->Initialize();

    system.AddLink(MooringLine1);
    system.AddLink(MooringLine2);
    system.AddLink(MooringLine3);
    system.AddLink(MooringLine4);

    // ----------------------------------------------
    // Motors
    // ----------------------------------------------
    // Barge - Crane base:
    auto rotmotor_crane = std::make_shared<ChLinkMotorRotationSpeed>();
    rotmotor_crane->Initialize(base_crane, barge, ChFrame<>(ChVector<>(0., 7.5, 2.)));
    system.Add(rotmotor_crane);
    auto rw_crane = std::make_shared<ChFunction_Const>(0.01);
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

    double dt = 0.0025;

    //system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);
    system.Initialize();

    auto app = FrIrrApp(system);
    app.AddTypicalCamera(irr::core::vector3df(20, 0, 20), irr::core::vector3df(0, 0, 3));
    //app.SetVideoframeSave(true);
    app.Run();

}