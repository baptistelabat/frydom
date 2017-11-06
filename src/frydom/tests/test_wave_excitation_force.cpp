//
// Created by frongere on 30/10/17.
//

#include "frydom/core/FrCore.h"
#include "frydom/hydrodynamics/FrHydroDB.h" // TODO: charger seulement un FrHydrodynamics
#include "frydom/hydrodynamics/FrLinearExcitationForce.h"

#include "frydom/environment/waves/FrWaveField.h"

#include "frydom/utils/FrIrrApp.h"

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace frydom;

int main(int argc, char* argv[]) {

    bool viz = false;

    // TODO
    // 1 creer le systeme: il doit avoir une surface libre, un seabed, un modele de maree, un modele de courant, un modele de vent
    FrOffshoreSystem system;

    auto freeSurface = system.GetFreeSurface();
    freeSurface->Initialize(-100, 100, 10);

    // Set the wave field
    auto waveField = std::make_shared<FrLinearWaveField>(REGULAR);
    waveField->SetMeanWaveDirection(0., DEG);
    waveField->SetRegularWaveHeight(1.);
    waveField->SetRegularWavePeriod(5., S);
    freeSurface->SetWaveField(waveField);

    // Get a waveProbe
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

    // Lier le corps BEM et le corps hydro
    cylinder->SetBEMBody(HDB.GetBody(0));

    // Creer un modele de force d'excitation et l'ajouter au corps hydro
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    cylinder->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);
    excForce->Initialize();
    excForce->Clear();
    excForce->Initialize();



    //

    if (viz) {
        auto app = FrIrrApp(system);
        app.Run();
    }


    return 0;
}