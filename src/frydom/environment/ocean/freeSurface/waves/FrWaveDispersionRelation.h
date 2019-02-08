// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRWAVEDISPERSIONRELATION_H
#define FRYDOM_FRWAVEDISPERSIONRELATION_H


#include <vector>
#include <cmath>

namespace frydom {

    double SolveWaveDispersionRelation(double water_height, double omega, double gravity);

    std::vector<double> SolveWaveDispersionRelation(double water_heigt, std::vector<double> omega, double gravity);

}  // end namespace frydom

#endif //FRYDOM_FRWAVEDISPERSIONRELATION_H
