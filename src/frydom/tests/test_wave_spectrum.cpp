//
// Created by frongere on 04/10/17.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    double hs = 3;
    double tp = 9;

    auto spectrum = std::make_unique<FrJonswapWaveSpectrum>(hs, tp);
    auto dir_model = std::make_unique<FrCos2sDirectionalModel>();

    spectrum->SetDirectionalModel(dir_model.release());

    uint nb_waves = 1000;
    double wmin = 1e-2;
    double wmax = 5.;

    auto S_w = spectrum->GetWaveAmplitudes(nb_waves, wmin, wmax);

    uint nb_dir = 180;
    double tmin = -M_PI;
    double tmax = M_PI;
    double tmean = 0.;

    auto S_w_theta = spectrum->GetWaveAmplitudes(nb_waves, wmin, wmax, nb_dir, tmin, tmax, tmean);



    return 0;
}