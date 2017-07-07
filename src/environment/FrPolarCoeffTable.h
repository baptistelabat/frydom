//
// Created by frongere on 03/07/17.
//

#ifndef FRYDOM_FRPOLARCOEFFTABLE_H
#define FRYDOM_FRPOLARCOEFFTABLE_H


//#include <vector>
#include <memory>
#include "core/FrConstants.h"

namespace frydom {
namespace environment {

    // Forward declaration
    class FrInterp1d;

    template <class Real=double>
    class FrPolarCoeffTable {

        // Lors de l'initialisation, il faut qu'un interpolateur soit disponible. On stocke les objets d'interpolation
        // et pas les donnees directement. -->

    private:
        std::shared_ptr<Real> xcoord;
        FrInterp1d* Cx_i;
        FrInterp1d* Cy_i;
        FrInterp1d* Cpsi_i;

    public:
        FrPolarCoeffTable() {};

        ~FrPolarCoeffTable() {};

        void SetInterpolator() {};

        /// To Initialize the table (may be overloaded following the way we load coefficients)
        void Initialize() {};
        void InitializeCx() {};
        void InitializeCy() {};
        void InitializeCpsi() {};

        /// Get the Cx coefficient given the field incidence
        Real Cx(const Real angle, FrAngleUnit unit=RAD) { return 0.; };

        /// Get the Cy coefficient given the field incidence
        Real Cy(const Real angle, FrAngleUnit unit=RAD) { return 0.; };

        /// Get the Cpsi coefficient given the field incidence
        Real Cpsi(const Real angle, FrAngleUnit unit=RAD) { return 0.; };

        /// Get Cx, Cy, Cpsi coefficients given de field incidence
        void GetCoeffs(const Real angle, Real &Cx, Real &Cy, Real &Cpsi, FrAngleUnit unit=RAD) {};


    };

}  // end namespace environment
}  // end namespace frydom


#endif //FRYDOM_FRPOLARCOEFFTABLE_H
