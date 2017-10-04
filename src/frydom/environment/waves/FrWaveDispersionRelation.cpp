//
// Created by frongere on 28/09/17.
//

#include "FrWaveDispersionRelation.h"

namespace frydom {

    double solve_dispersion_relation(const double water_height,
                                     const double omega,
                                     const double gravity) {

        const double tol = 1e-6;

        double wave_number;

        double w2_g = omega * omega / gravity;

        if (water_height <= 0.) {
            // Infinite water depth
            wave_number = w2_g;

        } else {

            double A = w2_g * water_height;
            double X = A; // Initial value

            double thX = tanh(X);
            double residue = X - X * thX;

            double Cdot;
            while (fabs(residue) > tol) {
                Cdot = -thX - X * (1 - thX * thX);

                X -= residue / Cdot;
                thX = tanh(X);
                residue = A - X * thX;
            }
            wave_number = X / water_height;

        }
        return wave_number;
    }

    std::vector<double> solve_dispersion_relation(const double water_heigt,
                                                  const std::vector<double> omega,
                                                  const double gravity) {

        std::vector<double> kk;
        double k;
        for (double w : omega) {
            k = solve_dispersion_relation(water_heigt, w, gravity);
            kk.push_back(k);
        }

        return kk;
    }

}  // end namespace frydom
