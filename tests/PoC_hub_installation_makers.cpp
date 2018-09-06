//
// Created by Lucas Letournel on 06/09/18.
//

#include "frydom/frydom.h"


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
    linearWaveField->SetRegularWaveHeight(0.5);
    linearWaveField->SetRegularWavePeriod(5);
    linearWaveField->SetMeanWaveDirection(180);
    //linearWaveField->GetWaveRamp()->Deactivate();
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
            1000);  // TODO: mettre en place une adaptation lorsqu'on a un residu du solveur trop important
    system.SetMaxItersSolverStab(200);
    system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<chrono::ChSolverMINRES>(system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    // -----------------------------------------------
    // Simulation
    // -----------------------------------------------

    double dt = 0.015;

    system.SetStep(dt);
    system.Initialize();
//    system.SetupInitial();

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
    barge->SetHydroMesh("Barge.obj", true);
    barge->SetPos(chrono::ChVector<double>(0., 0., 0.));
    system.AddBody(barge);
    //barge->SetBodyFixed(true);

    barge->SetInertiaXX(chrono::ChVector<double>(2.465e7,1.149e7,1.388e07));
    barge->SetInertiaXY(chrono::ChVector<double>(0, 0, 0));
    barge->SetMass(1137.6-180.6); // 1137.576e3 pour l'ensemble
    barge->SetMass(1137.6); // 1137.576e3 pour l'ensemble
    barge->SetCOG(chrono::ChVector<double>(0, 0, 0));
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
    HsDamping->SetSeakeepingDampings(1e6,1e10,1e10);
    HsDamping->SetManeuveuringDampings(1e6,1e6,1e10);
    HsDamping->SetName("linear damping");
    barge->AddForce(HsDamping);

    // Hydrodynamics
    system.SetHydroDB("Barge_frydom.hdb5");
    auto hydroMapIndex = system.GetHydroMapNb()-1;
    system.GetHydroMapper(hydroMapIndex)->Map(barge, 0);

    // Radiation model
    auto radModel = std::make_shared<FrRadiationConvolutionModel>(system.GetHydroDB(hydroMapIndex), &system);
    radModel->SetHydroMapIndex(hydroMapIndex);
//    radModel->AddRadiationForceToHydroBody(barge);

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

    return barge;
}