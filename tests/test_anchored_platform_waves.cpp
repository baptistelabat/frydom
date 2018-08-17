//
// Created by frongere on 17/01/18.
//

#include "frydom/frydom.h"
#include <thread>

using namespace frydom;

// FIXME: ces fonctions devraient se trouver dans le Initialize de FrOffshoreSystem !!
// FIXME: la provenance de python n'est pas maitrisee ici. En cas d'install Anaconda tel que preconise, cela pose des problemes
// par rapport aux packages python installes... (import zmq !!) --> voir si on peut pas faire en sorte dans le cmake
// que les preferences du PATH systeme soient prise en compte (.bashrc sous Linux/Anaconda)
void taskLaunchbroker(){
    std::system("python -u ./Broker/broker.py");
}

void taskLaunchlogger(){
    std::system("python -u ./Logger/logger.py");
}

int main(int argc, char* argv[]) {

//    std::thread t1 (taskLaunchbroker);
//    std::thread t2 (taskLaunchlogger);

    // ====================================================================================
    // Offshore system creation (it silently creates every environmental objects with default values
    // ====================================================================================
    FrOffshoreSystem system;

    // ====================================================================================
    // Environment settings
    // ====================================================================================

    // Setting the freesurface representation
    auto freeSurface = system.GetEnvironment()->GetFreeSurface();
//    freeSurface->SetGridType(FrFreeSurface::CARTESIAN);
//    freeSurface->SetGrid(-100, 100, 10);
    freeSurface->SetGridType(FrFreeSurface::CARTESIAN);
//    freeSurface->SetGrid(0, 0, 300, 25, 30);
    freeSurface->SetGrid(-200, 200, 5, -150, 150, 5);

    // Setting the wave field
    freeSurface->SetLinearWaveField(LINEAR_IRREGULAR);
//    freeSurface->UpdateAssetON(); // Comment if you don't want the free surface asset to be updated during the visualisation

    system.GetEnvironment()->GetSeabed()->SetGrid(-500,500,50,-500,500,50);
    system.GetEnvironment()->GetSeabed()->SetDepth(-225);


    auto linearWaveField = freeSurface->GetLinearWaveField();
    linearWaveField->SetWaveSpectrum(JONSWAP);
    // TODO: voir si on peut modifier avec la HDB...
    linearWaveField->SetMeanWaveDirection(0., DEG);  // TODO: utiliser aussi les conventions NORTH/GOTO/NED etc...
    linearWaveField->SetWavePulsations(0.05, 3.14, 80);
    linearWaveField->GetWaveSpectrum()->SetHs(4.); // Un peu de violence que diable !
    linearWaveField->GetWaveSpectrum()->SetTp(10.);

    linearWaveField->GetWaveRamp()->SetDuration(1.);
    linearWaveField->GetWaveRamp()->SetIncrease();

    // Set the current properties
    system.GetEnvironment()->GetCurrent()->Set(NORTH, 45., NED, GOTO, KNOT);

    // Set the wind properties
    system.GetEnvironment()->GetWind()->Set(NORTH_WEST, 10., NED, GOTO, KNOT); // FIXME: pas encore actif...

    // ====================================================================================
    // Creating the platform
    // ====================================================================================
    auto platform = std::make_shared<FrShip>();


    platform->SetName("DeepSea Stavanger");
    platform->SetHydroMesh("GVA7500.obj", true);

    // Data for current model
    platform->SetLateralUnderWaterArea(1680.);
    platform->SetTransverseUnderWaterArea(861.5);
    platform->SetLpp(136.);

    // TODO: faire en sorte de ne pas avoir a construire un ChVector !
    platform->SetInertiaXX(chrono::ChVector<double>(1.02e11 * 2, 1.34e11 * 2, 1.28e11 * 2)); // Attention, le *2 partout ici est pour emuler la masse ajoutee...
    platform->SetInertiaXY(chrono::ChVector<double>(-9.79e3 * 2, 4.73e3 * 2, 1.71e2 * 2));
    platform->SetMass(69.892e6 * 2); // TODO: Caler avec Camille
    // TODO: faire en sorte de ne pas avoir a construire un ChVector !
    platform->SetCOG(chrono::ChVector<double>(0, 0, 10.15)); // TODO: Caler avec Camille

//    platform->Set3DOF_ON(); // Activate if you want it to stay in the horizontal plane
    system.AddBody(platform); // Important

    // ====================================================================================
    // Adding forces to the platform
    // ====================================================================================

    // Linear Hydrostatics
    // -------------------
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    hstForce->GetStiffnessMatrix()->SetDiagonal(1.29e7, 4.2e7, 4.35e9);
    hstForce->GetStiffnessMatrix()->SetNonDiagonal(-1.97e3, -3.04e3, -3.69e4);
    platform->AddForce(hstForce);  // Comment to remove

    // Creating linear hydrodynamics database and mapping
    // --------------------------------------------------
    auto HDB = LoadHDB5("DeepSeaStavanger.hdb5");  // Hydro database

    // Mapping
    // TODO: reprendre globalement le fonctionnement du mapper pour simplifier
    auto hydroMapper = HDB.GetMapper();
    system.SetHydroMapper(hydroMapper);
    hydroMapper->Map(platform, 0);

    auto radiationModel = std::make_shared<FrRadiationConvolutionModel>(&HDB, &system); // TODO: changer en linearHydroModel...

    // Linear Excitation force
    // -----------------------
    // FIXME : pour l'excitation, ce qui compte pour la direction de la houle, c'est la direction relative de chaque composante
    // par rapport au X local de la plateforme. Il faut donc a priori remettre a jour le steadyForce de la force d'excitation
    // avec la direction relative instantanée !!!
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    platform->AddForce(excForce);  // Comment to remove
    // Creating a wave probe from waveField
    auto waveProbe = linearWaveField->NewWaveProbe(0, 0); // TODO: doit etre genere par linearHydroModel
    excForce->SetWaveProbe(waveProbe); // WaveProbe devrait etre genere en interne de linearHydroModel
    // TODO: ajouter un flag pour que la position du waveProbe soit mise a jour avec le position du corps.... (valable seulement en config sans interaction hydro de radiation...)


    // Linear Radiation force
    // ----------------------
    radiationModel->AddRadiationForceToHydroBody(platform);  // Comment to remove
    // FIXME: Attention, pas encore de masse ajoutee dans les calculs. Pour le moment, on ne peut que en ajouter artificiellemet.
    // L'integration dans le modele multicorps de chrono reste un point dur... (pas les termes diagonaux mais les couplages...)

    // Linear additional damping
    // -------------------------
    auto hydroDampings = std::make_shared<FrLinearDamping>();
    // TODO: faire en sorte de ne pas avoir a construire un ChVector !
    hydroDampings->SetManeuveuringDampings(chrono::ChVector<double>(1e7, 1e7, 1e8)); // A caler manuellement pour stabiliser...
    platform->AddForce(hydroDampings);

    // Second order wave drift
    // -----------------------
    // TODO


    // Current force
    // -------------
    auto currentForce = std::make_shared<FrCurrentForce>("PolarCurrentCoeffs_NC.yml");
    platform->AddForce(currentForce);


    // Wind Force
    // ----------
    // TODO

    // ====================================================================================
    // Mooring system definition
    // ====================================================================================
    // Parameters
    double a = 68;
    double b = 48;
    double c = 23;
    double h = 200;
    double Lu = 600;

    double l = sqrt( 0.5 * (Lu*Lu - h*h) );

    // TODO: avoir la possibilite de specifier directement les coordonnees dans les constructeurs de fairlead et anchor

    auto F1 = chrono::ChVector<double>(a, b, -c);
    auto A1 = chrono::ChVector<double>(a+l, b+l, -c-h);

    auto F2 = chrono::ChVector<double>(-a, b, -c);
    auto A2 = chrono::ChVector<double>(-a-l, b+l, -c-h);

    auto F3 = chrono::ChVector<double>(-a, -b, -c);
    auto A3 = chrono::ChVector<double>(-a-l, -b-l, -c-h);

    auto F4 = chrono::ChVector<double>(a, -b, -c);
    auto A4 = chrono::ChVector<double>(a+l, -b-l, -c-h);

    double alpha = 10;

    double L = (1.+alpha/100.) * Lu;


    // Catenary Line definition
    // ------------------------
    bool elastic = true; // TODO: creer une classe line properties...
    auto u = chrono::ChVector<double>(0, 0, -1);
    double q = 1000;
    double EA = 1e10;
    double A = 0.05;
    double E = EA/A;

    // TODO: pour les ancres, avoir un corps fixe pour le seabed qui doit etre le corps fixe de systeme en std::shared_ptr...
    // Du coup, avoir la possibilite de faire un seabed->AddAnchor...

    // Line 1
    auto fairlead_1 = platform->CreateNode(F1);
    auto anchor_1 = std::make_shared<FrNode>();
    anchor_1->SetBody(system.GetWorldBodyPtr());
    auto anchor_1_pos = chrono::ChCoordsys<double>();
    anchor_1_pos.pos = A1;
    anchor_1->Impose_Abs_Coord(anchor_1_pos);

    auto line_1 = std::make_shared<FrCatenaryLine>(fairlead_1, anchor_1, elastic, E, A, L, q, u);
    line_1->Initialize();
    system.AddLink(line_1);

    // Line 2
    auto fairlead_2 = platform->CreateNode(F2);
    auto anchor_2 = std::make_shared<FrNode>();
    anchor_2->SetBody(system.GetWorldBodyPtr());
    auto anchor_2_pos = chrono::ChCoordsys<double>();
    anchor_2_pos.pos = A2;
    anchor_2->Impose_Abs_Coord(anchor_2_pos);

    auto line_2 = std::make_shared<FrCatenaryLine>(fairlead_2, anchor_2, elastic, E, A, L, q, u);
    line_2->Initialize();
    system.AddLink(line_2);

    // Line 3
    auto fairlead_3 = platform->CreateNode(F3);
    auto anchor_3 = std::make_shared<FrNode>();
    anchor_3->SetBody(system.GetWorldBodyPtr());
    auto anchor_3_pos = chrono::ChCoordsys<double>();
    anchor_3_pos.pos = A3;
    anchor_3->Impose_Abs_Coord(anchor_3_pos);

    auto line_3 = std::make_shared<FrCatenaryLine>(fairlead_3, anchor_3, elastic, E, A, L, q, u);
    line_3->Initialize();
    system.AddLink(line_3);

    // Line 4
    auto fairlead_4 = platform->CreateNode(F4);
    auto anchor_4 = std::make_shared<FrNode>();
    anchor_4->SetBody(system.GetWorldBodyPtr());
    auto anchor_4_pos = chrono::ChCoordsys<double>();
    anchor_4_pos.pos = A4;
    anchor_4->Impose_Abs_Coord(anchor_4_pos);

    auto line_4 = std::make_shared<FrCatenaryLine>(fairlead_4, anchor_4, elastic, E, A, L, q, u);
    line_4->Initialize();
    system.AddLink(line_4);


    // ====================================================================================
    // System (and everything attached) initialization (Mandatory)
    // ====================================================================================
    system.Initialize();

    // ====================================================================================
    // Irrlicht visualization
    // ====================================================================================
    auto app = FrIrrApp(system, 150);

    double timeStep = 0.02;

    double recordingFPS = 30;

    // Video generation
    bool generateVideo = false;
    app.SetVideoframeSave(generateVideo);  // TODO: decelencher la generation de la video de maniere auto en post-processing...
    app.SetVideoframeSaveInterval(std::min<int>(1, (int)(1 / (recordingFPS * timeStep)) ));

    app.SetTimestep(timeStep);

    app.Run();

    return 0;
}