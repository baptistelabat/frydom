//
// Created by frongere on 03/07/17.
//

#ifndef FRYDOM_FRPOLARCOEFFTABLE_H
#define FRYDOM_FRPOLARCOEFFTABLE_H

#include "../core/FrConstants.h"

namespace frydom {
namespace environment {

    // Forward declaration
    class FrInterpolator;

    class FrPolarCoeffTable {

    private:
        double *cx[];
        double *cy[];
        double *cpsi[];

    public:
        FrPolarCoeffTable() {};

        ~FrPolarCoeffTable() {};

        /// To Initialize the table (may be overloaded following the way we load coefficients)
        void Initialize() {};

        /// Get the Cx coefficient given the field incidence
        double Cx(double angle, FrAngleUnit unit=RAD) { return 0.; };

        /// Get the Cy coefficient given the field incidence
        double Cy(double angle, FrAngleUnit unit=RAD) { return 0.; };

        /// Get the Cpsi coefficient given the field incidence
        double Cpsi(double angle, FrAngleUnit unit=RAD) { return 0.; };

        /// Get Cx, Cy, Cpsi coefficients given de field incidence
        void GetCoeffs(double angle, double &Cx, double &Cy, double &Cpsi, FrAngleUnit unit=RAD) {};


    };

}  // end namespace environment
}  // end namespace frydom


#endif //FRYDOM_FRPOLARCOEFFTABLE_H
