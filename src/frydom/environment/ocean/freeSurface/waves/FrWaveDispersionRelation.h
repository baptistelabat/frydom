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


#ifndef FRYDOM_FRWAVEDISPERSIONRELATION_H
#define FRYDOM_FRWAVEDISPERSIONRELATION_H


//#include <vector>
//#include <cmath>

namespace frydom {

    double SolveWaveDispersionRelation(double water_height, double omega, double gravity);

    std::vector<double> SolveWaveDispersionRelation(double water_heigt, std::vector<double> omega, double gravity);

}  // end namespace frydom

#endif //FRYDOM_FRWAVEDISPERSIONRELATION_H
