//
// Created by frongere on 29/09/17.
//

#include <iostream>
#include "frydom/environment/waves/FrWaveDispersionRelation.h"

using namespace frydom;


int main(int argc, char* argv[]) {

    double water_height = 5;
    double omega = 2.;
    double grav = 9.81;


    double k = frydom::solve_dispersion_relation(water_height, omega, grav);

    std::cout << "omega = " << omega << std::endl << "k = " << k << std::endl << "w2/g = " << omega*omega / grav << std::endl;


    return 0;
}
