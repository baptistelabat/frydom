//
// Created by frongere on 30/10/17.
//

//#include "frydom/core/FrHydroBody.h"
#include <frydom/hydrodynamics/FrHydroDB.h>
#include <frydom/core/FrOffshoreSystem.h>
#include <frydom/hydrodynamics/FrLinearExcitationForce.h>
#include <frydom/core/FrShip.h>
#include <frydom/environment/waves/FrFlatFreeSurface.h>
#include "frydom/environment/waves/FrWaveField.h"
//#include "frydom/environment/waves/FrWaveProbe.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace frydom;

int main(int argc, char* argv[]) {

    // The system
    FrOffshoreSystem system; // TODO: lors de la creation d'un offshoresystem, il devrait y avoir par defaut une surface libre flat...
    // FIXME: -> comment faire pour le 3DOF dans le cas d'une SL non flat ? --> pa de sens pour le 3DOF a priori...

    auto free_surface = std::make_unique<frydom::environment::FrFlatFreeSurface>(0.);
    free_surface->Initialize(-400, 400, 200, -100, 100, 100);

    // Creating a regular wave field
    auto regularWaveField = std::make_shared<FrRegularLinearWaveField>(9, 3, 0);

    free_surface->SetWaveField(regularWaveField);

    system.setFreeSurface(free_surface.release());



    // TODO: Il faut que le wavefield soit integre a la free surface...


    // Loading a hydrodynamic database
    auto HDB = LoadHDB5("../tools/frydom_hdb.h5");

    // Creating a hydrodynamic body
    auto hydroBody = std::make_shared<FrShip>();

    // Linking the HDB to physical body
    hydroBody->SetBEMBody(HDB.GetBody(0));
    system.AddBody(hydroBody);
    hydroBody->Set3DOF_ON();

    // =================================================================================================================
    // Regular Wave field
    // =================================================================================================================



    // Creating a wave probe
    auto waveProbe1 = regularWaveField->NewWaveProbe(0, 0);

    // Testing
    auto time = linspace<double>(0., 300., 1000);


    std::vector<double> elev1;
    for (auto t: time) {
        regularWaveField->UpdateTime(t);
//        std::cout << waveProbe1->GetElevation() << std::endl;
        elev1.push_back(waveProbe1->GetElevation());
    }

    // Creating an excitation force
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    hydroBody->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe1);

    for (auto t: time) {
        system.SetChTime(t);
        system.Update();
    }




//    plt::plot(time, elev1);
//    plt::show();

//    // =================================================================================================================
//    // Irregular wave field
//    // =================================================================================================================
//
//    // Creating an irregular wave field
//    auto waveSpectrum = std::make_unique<FrJonswapWaveSpectrum>(3, 9);
//
//    auto irregularWaveField = std::make_shared<FrIrregularLinearWaveField>(80, 0.1, 2., 0., waveSpectrum.release());
//
//    // Creating a wave probe
//    auto waveProbe2 = irregularWaveField->NewWaveProbe(0, 0);
//
//    // Testing
//    std::vector<double> elev2;
//    for (auto t: time) {
//        irregularWaveField->UpdateTime(t);
////        std::cout << waveProbe2->GetElevation() << std::endl;
//        elev2.push_back(waveProbe2->GetElevation());
//    }
////    plt::plot(time, elev2);
////    plt::show();
//
//    // =================================================================================================================
//    // Irregular wave field with directional spreading
//    // =================================================================================================================
//
//    // Creating the irregular wave field
//    auto waveSpectrum2 = std::make_unique<FrJonswapWaveSpectrum>(3, 9);
//
//    // Creating a directional model and registering it into the wave spectrum
//    auto directionalModel = std::make_unique<FrCos2sDirectionalModel>();
//    waveSpectrum2->SetDirectionalModel(directionalModel.release());
//
//    // Creating a directional wave field
//    auto directionalWaveField = std::make_shared<FrDirectionalLinearWaveField>(80, 0.1, 2.,
//                                                                               0.,
//                                                                               40, -180, 175,
//                                                                               waveSpectrum2.release());  // TODO: verifier l'unite d'angle !!!
//
//    // Creating a wave probe
//    auto waveProbe3 = directionalWaveField->NewWaveProbe(0, 0);
//
//    // Testing
//    std::vector<double> elev3;
//    for (auto t: time) {
//        directionalWaveField->UpdateTime(t);
////        std::cout << waveProbe3->GetElevation() << std::endl;
//        elev3.push_back(waveProbe3->GetElevation());
//    }
//
//    plt::plot(time, elev3);
//    plt::show();


    return 0;
}