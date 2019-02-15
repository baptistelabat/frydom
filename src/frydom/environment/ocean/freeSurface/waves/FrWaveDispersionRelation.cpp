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


#include "FrWaveDispersionRelation.h"

namespace frydom {

    /// Solve the water waves dispersion relation in finite water depth
    /// Solve the equation w**2 = gk * tanh(kh) <=>  A = w**2 * h /g = X * tanh(X) with X = kh along with the
    /// Newton-Raphson algorithm
    double SolveWaveDispersionRelation(const double water_height,
                                       const double omega,
                                       const double gravity) {

        static const double tol = 1e-6;

        double wave_number;

        double w2_g = omega * omega / gravity;

        if (water_height <= 0.) {
            // Infinite water depth
            wave_number = w2_g;

        } else {
            // Newton-Raphson iterations

            double A = w2_g * water_height;
            double X = A; // Initial value as deep water wave number (X = kh)

            double thX = tanh(X);
            double residue = A - X * thX;

            double Cdot;  // Derivative
            while (fabs(residue) > tol) {
                Cdot = -thX - X * (1 - thX * thX);

                X -= residue / Cdot; // Update

                thX = tanh(X);
                residue = A - X * thX;
            }
            wave_number = X / water_height;

        }
        return wave_number;
    }

    std::vector<double> SolveWaveDispersionRelation(const double water_heigt,
                                                    const std::vector<double> omega,
                                                    const double gravity) {

        std::vector<double> kVect;
        double k;
        for (double w : omega) {
            k = SolveWaveDispersionRelation(water_heigt, w, gravity);
            kVect.push_back(k);
        }

        return kVect;
    }

}  // end namespace frydom
