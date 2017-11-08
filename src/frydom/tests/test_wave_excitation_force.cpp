//
// Created by frongere on 30/10/17.
//

#include "frydom/core/FrCore.h"
#include "frydom/hydrodynamics/FrHydroDB.h" // TODO: charger seulement un FrHydrodynamics
#include "frydom/hydrodynamics/FrLinearExcitationForce.h"

#include "frydom/environment/waves/FrWaveField.h"

#include "frydom/utils/FrIrrApp.h"


using namespace frydom;

int main(int argc, char* argv[]) {


    // TODO
    // 1 creer le systeme: il doit avoir une surface libre, un seabed, un modele de maree, un modele de courant, un modele de vent

    // TODO: le systeme devrait etre un singleton de maniere a ce que chaque objet necessitant un enregistrement aupres de systeme
    // puisse le faire de son propre chef...
    // Du coup, systeme devrait etre cree lors de l'appel a un FRyDoM_INIT()
    // Est ce qu'on ne peut pas definir un objet frydom qui soit un singleton et qui donne alors access au systeme embarque ??? --> mieux !!
    FrOffshoreSystem system;

    auto freeSurface = system.GetFreeSurface();
    freeSurface->Initialize(-50, 50, 2.5, -50, 50, 100);

    // Set the wave field
//    freeSurface->SetLinearWaveField(LINEAR_REGULAR);
//    auto waveField = freeSurface->GetLinearWaveField();
//    waveField->SetMeanWaveDirection(0., DEG);
//    waveField->SetRegularWaveHeight(0.5);
//    waveField->SetRegularWavePeriod(5., S);


    freeSurface->SetLinearWaveField(LINEAR_IRREGULAR);
    auto waveField = freeSurface->GetLinearWaveField();
    waveField->SetMeanWaveDirection(0., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
    double wmin = 0.2;
    double wmax = 2.;
    unsigned int nbFreq = 70;
    waveField->SetWavePulsations(wmin, wmax, nbFreq, RADS);

    freeSurface->UpdateAssetON();

//    freeSurface->SetLinearWaveField(LINEAR_DIRECTIONAL);
//    auto waveField = freeSurface->GetLinearWaveField();
//    waveField->SetMeanWaveDirection(0., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
//    double wmin = 0.2;
//    double wmax = 2.;
//    unsigned int nbFreq = 70;
//    waveField->SetWavePulsations(wmin, wmax, nbFreq, RADS);


//    waveField->GetWaveSpectrum()->Eval(1.);
//    waveField->SetRegularWaveHeight(0.5);
//    waveField->SetRegularWavePeriod(5., S);
//    waveField->GetWaveSpectrum()->



    // Get a waveProbe for excitation force on body
    auto waveProbe = waveField->NewWaveProbe(0, 0);


    // 2 creer un corps hydro
    auto cylinder = std::make_shared<FrHydroBody>();
    cylinder->SetName("Cylinder");
    cylinder->SetHydroMesh("../src/frydom/tests/data/Cylinder.stl", true);
    cylinder->SetLateralUnderWaterArea(0.);
    cylinder->SetTransverseUnderWaterArea(0.);
    cylinder->SetLpp(0.);
    cylinder->SetInertiaXX(chrono::ChVector<double>(0, 0, 0));
    cylinder->SetMass(0.);


    // Importer une base de donnees hydro
    FrHydroDB HDB = LoadHDB5("../tools/frydom_hdb.h5");

    // Linking the physical floating body to its counterpart from the HDB
    cylinder->SetBEMBody(HDB.GetBody(0));

    // Creer un modele de force d'excitation et l'ajouter au corps hydro
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    cylinder->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);
    excForce->Initialize();
    excForce->Clear();
    excForce->Initialize();


    auto app = FrIrrApp(system, 70);
    app.Run();


    return 0;
}