//
// Created by frongere on 06/10/17.
//

//#include <iostream>
#include "frydom/environment/waves/FrWaveField.h"

using namespace frydom;

int main(int argc, char* argv[]) {


    // Regular linear wave field
    auto regular_wave_field = FrRegularLinearWaveField(9, 3, 0.);

    // Computing on a grid
//    auto xVect = linspace<double>(-1000., 1000., 100);
//    auto yVect = linspace<double>(-1000., 1000., 100);
//
//    regular_wave_field.UpdateTime(0.);
//    auto fse = regular_wave_field.GetFreeSurfaceElevationGrid(xVect, yVect);


    // Irregular linear wave field
    auto jws = std::make_unique<FrJonswapWaveSpectrum>(3, 9); // FIXME: attention a l'ordre des arguments qui doit etre consistant !!
    auto irregular_wave_field = FrIrregularLinearWaveField(100, 0.01, 2.5, 0., jws.release());

    irregular_wave_field.UpdateTime(0.);

    std::cout << irregular_wave_field.GetFreeSurfaceElevation(0, 0);


    // Directional wave field
//    auto jws2 = std::make_unique<FrJonswapWaveSpectrum>(3, 9);
//    auto dirjws = std::make_unique<FrDirectionalWaveSpectrum>(jws2.release());




//    std::cout << regular_wave_field.GetFreeSurfaceElevation(0, 0) << "\n";
//    regular_wave_field.UpdateTime(9./4);
//    std::cout << regular_wave_field.GetFreeSurfaceElevation(0, 0);


//    auto wave_field = FrIrregularLinearWaveField(1000, 0.01, 3., 0., jws.release());






    return 0;
}