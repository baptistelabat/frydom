//
// Created by frongere on 04/10/17.
//

//#include "frydom/misc/FrLinspace.h"
#include "frydom/environment/waves/FrWaveSpectrum.h"


using namespace frydom;

int main(int argc, char* argv[]) {

    double hs = 3;
    double tp = 9;

    auto spectrum = std::make_unique<FrJonswapWaveSpectrum>(hs, tp);

    auto dir_spectrum = FrCos2sDirectionalWaveSpectrum(spectrum.release());

    auto w = linspace(0., 5., 1000);
    auto theta = linspace(-M_PI, M_PI, 360);

    auto S_w_theta = dir_spectrum.Eval(w, theta, 0.);


    auto wave_ampl = dir_spectrum.GetWaveAmplitudes(100, 0.01, 3., 180, -M_PI, M_PI, 0.);



    return 0;
}