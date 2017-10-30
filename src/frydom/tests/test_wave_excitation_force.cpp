//
// Created by frongere on 30/10/17.
//

#include "frydom/environment/waves/FrWaveField.h"
//#include "frydom/environment/waves/FrWaveProbe.h"


using namespace frydom;

int main(int argc, char* argv[]) {

    // =================================================================================================================
    // Regular Wave field
    // =================================================================================================================

    // Creating a regular wave field
    auto regularWaveField = std::make_shared<FrRegularLinearWaveField>(9, 3, 0);

    // Creating a wave probe
    auto waveProbe1 = regularWaveField->NewWaveProbe(0, 0);

    // Testing
    auto time = linspace<double>(0., 10., 1000);

//    for (auto t: time) {
//        regularWaveField->UpdateTime(t);
//        std::cout << waveProbe1->GetElevation() << std::endl;
//    }

    // =================================================================================================================
    // Irregular wave field
    // =================================================================================================================

    // Creating an irregular wave field
    auto waveSpectrum = std::make_unique<FrJonswapWaveSpectrum>(3, 9);

    auto irregularWaveField = std::make_shared<FrIrregularLinearWaveField>(80, 0.1, 2., 0., waveSpectrum.release());

    // Creating a wave probe
    auto waveProbe2 = irregularWaveField->NewWaveProbe(0, 0);

    // Testing
//    for (auto t: time) {
//        irregularWaveField->UpdateTime(t);
//        std::cout << waveProbe2->GetElevation() << std::endl;
//    }

    // =================================================================================================================
    // Irregular wave field with directional spreading
    // =================================================================================================================

    // Creating the irregular wave field
    auto waveSpectrum2 = std::make_unique<FrJonswapWaveSpectrum>(3, 9);

    // Creating a directional model and registering it into the wave spectrum
    auto directionalModel = std::make_unique<FrCos2sDirectionalModel>();
    waveSpectrum2->SetDirectionalModel(directionalModel.release());

    // Creating a directional wave field
    auto directionalWaveField = std::make_shared<FrDirectionalLinearWaveField>(80, 0.1, 2.,
                                                                               0.,
                                                                               40, -180, 175,
                                                                               waveSpectrum2.release());  // TODO: verifier l'unite d'angle !!!

    // Creating a wave probe
    auto waveProbe3 = directionalWaveField->NewWaveProbe(0, 0);

    // Testing
    std::vector<double> elev3;
    for (auto t: time) {
        directionalWaveField->UpdateTime(t);
        std::cout << waveProbe3->GetElevation() << std::endl;
        elev3.push_back(waveProbe3->GetElevation());
    }





    return 0;
}