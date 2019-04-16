// ==========================================================================
// FRyDoM - frydom-ce.org
// 
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
// 
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
// 
// ==========================================================================

#include "frydom/frydom.h"

using namespace frydom;


int main(int argc, char* argv[]) {

    double water_height = 5;
    double omega = 2.;
    double grav = 9.81;


    double k = frydom::SolveWaveDispersionRelation(water_height, omega, grav);

    std::cout << "omega = " << omega << std::endl << "k = " << k << std::endl << "w2/g = " << omega*omega / grav << std::endl;


    return 0;
}
