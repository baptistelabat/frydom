//
// Created by frongere on 28/09/17.
//

#ifndef FRYDOM_FRWAVEDISPERSIONRELATION_H
#define FRYDOM_FRWAVEDISPERSIONRELATION_H


#include <vector>
#include <cmath>

namespace frydom {

    double SolveWaveDispersionRelation(double water_height, double omega, double gravity);

    std::vector<double> SolveWaveDispersionRelation(double water_heigt, std::vector<double> omega, double gravity);

}  // end namespace frydom

#endif //FRYDOM_FRWAVEDISPERSIONRELATION_H
