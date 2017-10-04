//
// Created by frongere on 28/09/17.
//

#ifndef FRYDOM_FRWAVEDISPERSIONRELATION_H
#define FRYDOM_FRWAVEDISPERSIONRELATION_H


#include <vector>
#include <cmath>

namespace frydom {

    double solve_dispersion_relation(const double water_height,
                                     const double omega,
                                     const double gravity);

    std::vector<double> solve_dispersion_relation(const double water_heigt, const std::vector<double> omega, const double gravity);

}  // end namespace frydom

#endif //FRYDOM_FRWAVEDISPERSIONRELATION_H
