//
// Created by frongere on 30/10/17.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {


    // TODO
    // 1 creer le systeme: il doit avoir une surface libre, un seabed, un modele de maree, un modele de courant, un modele de vent

    // TODO: le systeme devrait etre un singleton de maniere a ce que chaque objet necessitant un enregistrement aupres de systeme
    // puisse le faire de son propre chef...
    // Du coup, systeme devrait etre cree lors de l'appel a un FRyDoM_INIT()
    // Est ce qu'on ne peut pas definir un objet frydom qui soit un singleton et qui donne alors access au systeme embarque ??? --> mieux !!
    FrOffshoreSystem mySystem;


    // Free surface appearance
    auto freeSurface = mySystem.GetEnvironment()->GetFreeSurface();
    freeSurface->SetGridType(FrFreeSurface::POLAR);
    freeSurface->SetGrid(0, 0, 40, 20, 36);

    // TODO: permettre de regler le hs et le tp...
    freeSurface->SetLinearWaveField(LINEAR_IRREGULAR);
    auto waveField = freeSurface->GetLinearWaveField();
    waveField->SetMeanWaveDirection(21., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
    double wmin = 0.1;
    double wmax = 3.;
    unsigned int nbFreq = 80;
    waveField->SetWavePulsations(wmin, wmax, nbFreq, RADS);
    waveField->GetWaveSpectrum()->SetHs(1.);  // 1
    waveField->GetWaveSpectrum()->SetTp(5.5);  // 7

    waveField->GetWaveRamp()->SetDuration(5.);
    waveField->GetWaveRamp()->SetIncrease();

    // To see the free surface mesh updated
    freeSurface->UpdateAssetON();


    // Get a waveProbe for excitation force on body
    auto waveProbe = waveField->NewWaveProbe(0, 0); // TODO: il faut importer la position depuis la HDB... (position de mesure de houle)


    // 2 creer un corps hydro
    auto cylinder = std::make_shared<FrHydroBody>();
    cylinder->SetName("Cylinder");
    cylinder->SetHydroMesh("Cylinder.obj", true);
    cylinder->SetLateralUnderWaterArea(100.);
    cylinder->SetTransverseUnderWaterArea(100.);
    cylinder->SetLpp(10.);
    // TODO: Accepter 3 parametres plutot qu'un vecteur chrono
    cylinder->SetInertiaXX(chrono::ChVector<double>(8e7, 8e7, 1e6));
    cylinder->SetMass(795e3 * 2.);  // TODO : retirer le 2., c'est pour emuler une masse ajoutee...
    cylinder->SetCOG(chrono::ChVector<double>(0., 0., -7.5));
//    cylinder->SetBodyFixed(true);  // TODO: retirer
    mySystem.AddBody(cylinder);

    // ===========================================
    // Hydrodynamics
    // ===========================================

    // Set added mass


    // Hydrostatics
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    auto hstStiffness = hstForce->GetStiffnessMatrix();
    double K33 = 7.7922E+05;
    double K44 = 3.4146E+07;
    hstStiffness->SetDiagonal(K33, K44, K44);
    cylinder->AddForce(hstForce);

    // Linear Damping
    auto hydroDampingForce = std::make_shared<FrLinearDamping>();
//    hydroDampingForce->SetManeuveuringDampings(chrono::ChVector<double>(1e8, 1e8, 1e9));
//    hydroDampingForce->SetSeakeepingDampings(chrono::ChVector<double>(1e6, 1e9, 1e9));


    // Playing
    // TODO: accepter 3 arguments plutot que des vecteurs chrono
    hydroDampingForce->SetManeuveuringDampings(chrono::ChVector<double>(1e8, 1e8, 1e9));
//    hydroDampingForce->SetSeakeepingDampings(chrono::ChVector<double>(1e5, 5e10, 5e10));


    cylinder->AddForce(hydroDampingForce);


    // Linear Seakeeping forces
    // ========================

    // Hydro database loading
    // TODO: renommer frydom_hdb.h5 en cylinder.hdb5
    FrHydroDB HDB = LoadHDB5("frydom_hdb.h5");  // TODO: mettre la HDB dans system ??? (ca genere du coup le mapper en auto)

    // Mapping physical hydrodynamic bodies and BEM bodies
    auto hydroMapper = HDB.GetMapper();  // C'est instancie dans HDB, ne devrait pas, devrait l'etre dans le modele de force lineaire...
    mySystem.SetHydroMapper(hydroMapper);
    hydroMapper->Map(cylinder, 0);

    auto radModel = std::make_shared<FrRadiationConvolutionModel>(&HDB, &mySystem);  // TODO : Ajouter un addExcitationForceToHydroBody (et changer le nom de radiationModel...)




    // Creer un modele de force d'excitation et l'ajouter au corps hydro
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    cylinder->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);  // La wave probe devrait etre generee automatiquement par la classe FrLinearHydroModel car c'est une info de la HDB !!
//    excForce->Initialize();  // TODO: voir avoir une initialisation auto lors du lancement de la simu (avec un flag...)  --> OK fait

    // Radiation

    // TODO: creer le modele de radiation !!!



    radModel->AddRadiationForceToHydroBody(cylinder);
//    auto radForce = std::make_shared<FrRadiationConvolutionForce>();
//    cylinder->AddForce(radForce);
//    radForce->Initialize();


//    radModel->Initialize();  // Voir eventuellement a trigger l'initialisation lors de l'init de la premiere force qui y est rattache...



    // TODO: tester le solveur EULER_IMPLICIT_PROJECTED
    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::TRAPEZOIDAL);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::HHT);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::HEUN);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::RUNGEKUTTA45);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::EULER_EXPLICIT);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::LEAPFROG);
//    mySystem.SetTimestepperType(chrono::ChTimestepper::Type::NEWMARK);
    auto app = FrIrrApp(mySystem, 30);

    app.SetTimestep(0.02);
//    radForce->Initialize();


//    app.SetVideoframeSaveInterval(1);
//    app.SetVideoframeSave(true);
    mySystem.Initialize(); // Very important !!
    app.Run();  // TODO: remettre

//    char* path;
//    path = getenv("FRYDOM_BASE_DIR");
//    std::cout << path;



    return 0;
}