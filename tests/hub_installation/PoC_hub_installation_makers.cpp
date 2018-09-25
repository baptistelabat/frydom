//
// Created by Lucas Letournel on 06/09/18.
//

#include "frydom/frydom.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono_mumps/ChSolverMumps.h"


using namespace chrono;
using namespace frydom;

void make_environment(FrOffshoreSystem& system) {
    // ====================================================================================
    // Environment settings
    // ====================================================================================

    // Setting the freesurface representation
    auto freeSurface = system.GetEnvironment()->GetFreeSurface();

    // Setting the wave field
    freeSurface->SetLinearWaveField(LINEAR_REGULAR);
    freeSurface->UpdateAssetON(); // Comment if you don't want the free surface asset to be updated during the visualisation

    auto linearWaveField = freeSurface->GetLinearWaveField();
    linearWaveField->SetRegularWaveHeight(0.75);
    linearWaveField->SetRegularWavePeriod(6);
    linearWaveField->SetMeanWaveDirection(180);
//    linearWaveField->GetWaveRamp()->Deactivate();

//    system.GetEnvironment()->GetCurrent()->Set(WEST, 1.5, NED, COMEFROM, KNOT);
//    system.GetEnvironment()->GetWind()->Set(WEST, 10., NED, COMEFROM, KNOT);
}

void run_simulation(FrOffshoreSystem& system) {
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
            200);  // TODO: mettre en place une adaptation lorsqu'on a un residu du solveur trop important
    system.SetMaxItersSolverStab(200);
    system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<chrono::ChSolverMINRES>(system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);


//    auto mumps_solver = std::make_shared<chrono::ChSolverMumps>();
//    mumps_solver->SetSparsityPatternLock(true);
//    system.SetSolver(mumps_solver);



//    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

//    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
//    auto integrator = std::static_pointer_cast<chrono::ChTimestepperEulerImplicit>(system.GetTimestepper());

    system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<chrono::ChTimestepperHHT>(system.GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(8);
    integrator->SetAbsTolerances(5e-05,1.8e00);
    integrator->SetMode(chrono::ChTimestepperHHT::POSITION);
    integrator->SetModifiedNewton(false);
    integrator->SetScaling(true);


//    integrator->SetAlpha(-0.2);
//    integrator->SetMaxiters(50);
//    integrator->SetAbsTolerances(1e-1, 1e3);
//    integrator->SetMode(ChTimestepperHHT::ACCELERATION);
//    integrator->SetStepControl(false);
//    integrator->SetModifiedNewton(true);
//    integrator->SetScaling(true);
//    integrator->SetVerbose(verbose_integrator);

//    system.SetTimestepperType(chrono::ChTimestepper::Type::NEWMARK);
//    auto integrator = std::static_pointer_cast<chrono::ChTimestepperNewmark>(system.GetTimestepper());
//    integrator->SetVerbose(true);
//    integrator->SetMaxiters(8);
//    integrator->SetAbsTolerances(5e-05,1.8e00);
//    integrator->SetGammaBeta(0.75, 0.5);

//    integrator->SetVerbose(true);





    // -----------------------------------------------
    // Simulation
    // -----------------------------------------------

    double dt = 0.01;

    system.SetStep(dt);
    system.Initialize();
    system.SetupInitial();

    auto app = FrIrrApp(system, 75);
    //app.SetShowInfos(true);
    app.SetVideoframeSave(false);
    app.Run();
}



std::shared_ptr<FrHydroBody> make_barge(FrOffshoreSystem& system){
    // --------------------------------------------------
    // Barge model
    // --------------------------------------------------

    auto barge = std::make_shared<FrHydroBody>();
    barge->SetName("Barge");
    barge->Log().SetNameAndDescription("bargeLog","Message log of the ship");
    barge->SetHydroMesh("Barge.obj", true);
    barge->SetPos(chrono::ChVector<double>(0., 0., 0.));
    system.AddBody(barge);
    //barge->SetBodyFixed(true);

    barge->SetInertiaXX(chrono::ChVector<double>(2.465e7,1.149e8,1.388e08));
//    barge->SetInertiaXX(chrono::ChVector<double>(2.465e7 + 3.11432e+07,1.149e8 + 2.1652e+08,1.388e08 + 1.4624e+07));
//    barge->SetInertiaXX(chrono::ChVector<double>(20*3.11432e+07,20*2.1652e+08,20*1.4624e+07));
    barge->SetInertiaXY(chrono::ChVector<double>(0, 0.94e5, 0));
    barge->SetMass((1137.6-180.6)*1e3); // 1137.576e3 pour l'ensemble
    barge->SetMass(1137.6e3); // 1137.576e3 pour l'ensemble
//    barge->SetMass(1137.6e3 + 3.34317e+06); // 1137.576e3 pour l'ensemble
    barge->SetCOG(chrono::ChVector<double>(0, 0, 0));
    barge->SetEquilibriumFrame(WorldFixed,chrono::ChVector<>(0,0,0));
//    barge->Set3DOF_ON();

    // ====================================================================================
    // Adding forces to the platform
    // ====================================================================================

    // Linear Hydrostatics
    // -------------------
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    //hstForce->GetStiffnessMatrix()->SetDiagonal(5.62e6, 1.09e9, 5.63e9);
    hstForce->GetStiffnessMatrix()->SetDiagonal(5.62e6, 1.09e8, 5.63e8);
    hstForce->GetStiffnessMatrix()->SetNonDiagonal(0e3, 0e3, 0e4);
    hstForce->SetName("linear hydrostatic");
    barge->AddForce(hstForce);

    // Linear Damping

    auto HsDamping = std::make_shared<FrLinearDamping>();
    HsDamping->SetDiagonalDamping(1e6,1e6,1e6,1e10,1e10,1e10);
    HsDamping->SetName("linear damping");
//    barge->AddForce(HsDamping);

    // Hydrodynamics
    system.SetHydroDB("Barge_frydom.hdb5");
    auto hydroMapIndex = system.GetHydroMapNb()-1;
    system.GetHydroMapper(hydroMapIndex)->Map(barge, 0);

    // Radiation model
    auto radModel = std::make_shared<FrRadiationConvolutionModel>(system.GetHydroDB(hydroMapIndex), &system);
    radModel->SetHydroMapIndex(hydroMapIndex);
    radModel->AddRadiationForceToHydroBody(barge);

    // Wave Probe
    auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    auto waveProbe = waveField->NewWaveProbe(0,0);
    //auto waveProbe = std::make_shared<FrLinearWaveProbe>(0,0);
    waveProbe->Initialize();

    // Wave Excitation force
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    excForce->SetName("excitation force");
    barge->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);
    excForce->SetHydroMapIndex(hydroMapIndex);


    barge->Log().SetNameAndDescription("BargeLog","Message log of the ship");
    barge->SetLogDefault();

    for (auto force: barge->GetForceList()) {
        auto dforce = dynamic_cast<FrForce*>(force.get());
        barge->Log().AddField<hermes::Message>("force","-","external force", dforce->GetLog());
    }

    return barge;
}

void make_crane(FrOffshoreSystem& system, std::shared_ptr<FrBody>& base_crane, std::shared_ptr<FrBody>& arm_crane) {
    // --------------------------------------------------
    // Crane model
    // --------------------------------------------------

    auto ColorAsset_Red = std::make_shared<ChColorAsset>(ChColor(1.f, 0.f, 0.0f));

    base_crane = std::make_shared<FrBody>();
    base_crane->SetName("Base_crane");
    base_crane->SetVisuMesh("BaseCrane.obj");
    base_crane->SetPos(chrono::ChVector<double>(-7.5, 0, 1.15));
    base_crane->SetMass(140e3);
    base_crane->AddAsset(ColorAsset_Red);
    system.AddBody(base_crane);
    //base_crane->SetBodyFixed(true);

    arm_crane = std::make_shared<FrBody>();
    arm_crane->SetName("Arm_crane");
    arm_crane->SetVisuMesh("arm_crane.obj");
    arm_crane->SetPos(chrono::ChVector<double>(-5.5, 0, 2.5));
    arm_crane->SetRot(Q_from_AngAxis(-3*CH_C_PI_4/2, VECT_Y));
    arm_crane->SetMass(20e3);
    arm_crane->AddAsset(ColorAsset_Red);
    system.AddBody(arm_crane);
    //arm_crane->SetBodyFixed(true);

}

void make_hub(FrOffshoreSystem& system, std::shared_ptr<FrBody>& hub_box, std::shared_ptr<FrBody>& arm_crane) {

hub_box = std::make_shared<FrBody>();
hub_box->SetName("HubBox");
hub_box->SetVisuMesh("HubBox.obj");
hub_box->SetPos(chrono::ChVector<double>(8.27, 0, 2.1));
hub_box->SetMass(20.7e3);
system.AddBody(hub_box);
//hub_box->SetBodyFixed(true);


// ---------------------------------------------
// Hub Line
// ---------------------------------------------

auto A3_tige = arm_crane->CreateNode(ChVector<double>(36., 0., 0.));
auto A4_hub = hub_box->CreateNode(ChVector<double>(0., 0., 0.));
auto A5_hub = hub_box->CreateNode(ChVector<double>(0., 0., 0.));

// Line properties
double Lu = 33.6596;
auto u = chrono::ChVector<double>(0, 0, -1);
double q = 600;
double EA = 5e8;
double A = 0.05;
double E = EA / A;
double breakTensionAsset = 500000;

//auto Catenary = std::make_shared<FrCatenaryLine>(A3_tige, A4_hub, true, E, A, Lu, q, u);
//Catenary->SetBreakingTension(breakTensionAsset);
//Catenary->Initialize();
//system.AddLink(Catenary);

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


}

void make_mooring(FrOffshoreSystem& system, std::shared_ptr<FrHydroBody>& barge) {
    // ---------------------------------------------
    // Mooring Lines
    // ---------------------------------------------
    auto ColorAsset_Red = std::make_shared<chrono::ChColorAsset>(1.0f,0.f,0.f);

    auto buoy1 = std::make_shared<FrMooringBuoy>(0.75, 1e2, true, 1e3);
    buoy1->SetName("Buoy1");
    buoy1->SetPos(ChVector<>(-50, -25., 0));
    buoy1->SetCOG(ChVector<>(0, 0, 0));
    //buoy1->SetBodyFixed(true);
    buoy1->AddAsset(ColorAsset_Red);
    system.AddBody(buoy1);

    auto buoy2 = std::make_shared<FrMooringBuoy>(0.75, 1e2, true, 1e3);
    buoy2->SetName("Buoy2");
    buoy2->SetPos(ChVector<>(-50, 25., 0));
    buoy2->SetCOG(ChVector<>(0, 0, 0));
    //buoy2->SetBodyFixed(true);
    buoy2->AddAsset(ColorAsset_Red);
    system.AddBody(buoy2);

    auto buoy3 = std::make_shared<FrMooringBuoy>(0.75, 1e2, true, 1e3);
    buoy3->SetName("Buoy3");
    buoy3->SetPos(ChVector<>(50, -25., 0));
    buoy3->SetCOG(ChVector<>(0, 0, 0));
    //buoy3->SetBodyFixed(true);
    buoy3->AddAsset(ColorAsset_Red);
    system.AddBody(buoy3);

    auto buoy4 = std::make_shared<FrMooringBuoy>(0.75, 1e2, true, 1e3);
    buoy4->SetName("Buoy4");
    buoy4->SetPos(ChVector<>(50, 25., 0));
    buoy4->SetCOG(ChVector<>(0, 0, 0));
    //buoy4->SetBodyFixed(true);
    buoy4->AddAsset(ColorAsset_Red);
    system.AddBody(buoy4);

    /// Mooring lines length
    auto Lu = 95;
    auto u = chrono::ChVector<double>(0, 0, -1);
    auto q = 20.;
    auto EA = 5e6;
    auto A = 0.025;
    auto E = EA / A;
    double Lv = 42.5;
    auto breakTensionAsset = 50000;

    auto B1 = barge->CreateNode(ChVector<double>(-12.5, -5, 0.));
    auto B2 = barge->CreateNode(ChVector<double>(-12.5, 5, 0.));
    auto B3 = barge->CreateNode(ChVector<double>(12.5, -5, 0.));
    auto B4 = barge->CreateNode(ChVector<double>(12.5, 5, 0.));

    auto BB1 = buoy1->CreateNode(ChVector<double>(0, 0, 0));
    auto BB2 = buoy2->CreateNode(ChVector<double>(0, 0, 0));
    auto BB3 = buoy3->CreateNode(ChVector<double>(0, 0, 0));
    auto BB4 = buoy4->CreateNode(ChVector<double>(0, 0, 0));

    auto WB1 = system.GetWorldBody()->CreateNode(ChVector<double>(-125, -75, -30.));
    auto WB2 = system.GetWorldBody()->CreateNode(ChVector<double>(-125, 75, -30.));
    auto WB3 = system.GetWorldBody()->CreateNode(ChVector<double>(125, -75, -30.));
    auto WB4 = system.GetWorldBody()->CreateNode(ChVector<double>(125, 75, -30.));

    auto MooringLine1 = std::make_shared<FrCatenaryLine>(B1, BB1, true, E, A, Lv, q, u);
    auto MooringLine2 = std::make_shared<FrCatenaryLine>(B2, BB2, true, E, A, Lv, q, u);
    auto MooringLine3 = std::make_shared<FrCatenaryLine>(B3, BB3, true, E, A, Lv, q, u);
    auto MooringLine4 = std::make_shared<FrCatenaryLine>(B4, BB4, true, E, A, Lv, q, u);

    auto MooringLine1b = std::make_shared<FrCatenaryLine>(BB1, WB1, true, E, A, Lu, q, u);
    auto MooringLine2b = std::make_shared<FrCatenaryLine>(BB2, WB2, true, E, A, Lu, q, u);
    auto MooringLine3b = std::make_shared<FrCatenaryLine>(BB3, WB3, true, E, A, Lu, q, u);
    auto MooringLine4b = std::make_shared<FrCatenaryLine>(BB4, WB4, true, E, A, Lu, q, u);

//    MooringLine1->SetBreakingTension(breakTensionAsset);
//    MooringLine2->SetBreakingTension(breakTensionAsset);
//    MooringLine3->SetBreakingTension(breakTensionAsset);
//    MooringLine4->SetBreakingTension(breakTensionAsset);
//    MooringLine1b->SetBreakingTension(breakTensionAsset);
//    MooringLine2b->SetBreakingTension(breakTensionAsset);
//    MooringLine3b->SetBreakingTension(breakTensionAsset);
//    MooringLine4b->SetBreakingTension(breakTensionAsset);

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
}

void make_export_cable(FrOffshoreSystem& system, std::shared_ptr<FrBody>& hub_box) {
    // ---------------------------------------------
    // Export Cable
    // ---------------------------------------------
    auto A5_hub = hub_box->CreateNode(ChVector<double>(0., 0., 0.));

    // Line properties
    auto Lu = 155;
    auto u = chrono::ChVector<double>(0, 0, -1);
    auto q = 20;
    auto EA = 1e7;
    auto A = 0.05;
    auto E = EA / A;
    auto breakTensionAsset = 100000;

    auto Export = system.GetWorldBody()->CreateNode(ChVector<double>(0, 150, -30.));

//    auto DynamicLine = std::make_shared<FrDynamicCable>();
//    DynamicLine->SetStartingNode(Export);
//    DynamicLine->SetEndingNode(A5_hub);
//    DynamicLine->SetCableLength(Lu);
//    DynamicLine->SetNumberOfElements(20);
//    DynamicLine->SetLinearDensity(q);
//    DynamicLine->SetSectionArea(A);
//    DynamicLine->SetYoungModulus(E);
//    DynamicLine->SetRayleighDamping(0.01);
//    DynamicLine->Initialize();
//    system.Add(DynamicLine);

    auto ExportLine = std::make_shared<FrCatenaryLine>(Export, A5_hub, true, E, A, Lu, q, u);
    ExportLine->Initialize();
    system.AddLink(ExportLine);
}

void make_motors_crane(FrOffshoreSystem& system, std::shared_ptr<FrHydroBody>& barge, std::shared_ptr<FrBody>& base_crane, std::shared_ptr<FrBody>& arm_crane){
    // ----------------------------------------------
    // Motors
    // ----------------------------------------------
    // Barge - Crane base:
    auto rotmotor_crane = std::make_shared<ChLinkMotorRotationSpeed>();
    rotmotor_crane->Initialize(base_crane, barge, ChFrame<>(ChVector<>(-7.5, 0., 2.)));
    system.Add(rotmotor_crane);

    auto rw_crane = std::make_shared<ChFunction_Const>(RPM2RADS(0.5));
    rotmotor_crane->SetSpeedFunction(rw_crane);

    // Crane base - Crane arm
    auto rotmotor_arm = std::make_shared<ChLinkMotorRotationAngle>();
    rotmotor_arm->Initialize(arm_crane, base_crane, ChFrame<>(ChVector<>(-5.5, 0., 3.5), CH_C_PI_2, VECT_X));
    system.Add(rotmotor_arm);
    auto rw_arm = std::make_shared<ChFunction_Const>(0.);
    rotmotor_arm->SetAngleFunction(rw_arm);
}