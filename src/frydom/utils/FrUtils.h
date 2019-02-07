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


#ifndef FRYDOM_FRUTILS_H
#define FRYDOM_FRUTILS_H

#include <iostream>
#include <complex>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix.h"


namespace frydom {

    /// Print functions for chrono
    // TODO: placer ces fonctions dans un utilitaire autre que FrEigen !!
    // TODO : utiliser sprintf...
    template <class Real=double>
    std::ostream& operator<<(std::ostream& os, const chrono::ChMatrix<Real>& mat) {
        os << std::endl;
        for (int i=0; i<mat.GetRows(); ++i) {
            for (int j=0; j<mat.GetColumns(); j++) {
                os << mat.GetElement(i, j) << "\t";
            }
            os << std::endl;
        }
        os << std::endl;
        return os;
    }

    template <class Real=double>
    std::ostream& operator<<(std::ostream& os, const chrono::ChVector<Real>& vect) {
        os << std::endl << vect.x() << std::endl << vect.y() << std::endl << vect.z() << std::endl << std::endl;
        return os;
    }

    template <class Real=double>
    chrono::ChVector<Real> ChReal(const chrono::ChVector<std::complex<Real>>& vect) { // TODO: changer les noms en Ch --> piegeux avec les classes Chrono !!
        chrono::ChVector<Real> reVect;
        reVect.x() = std::real(vect.x());
        reVect.y() = std::real(vect.y());
        reVect.z() = std::real(vect.z());
        return reVect;
    }

    template <class Real=double>
    chrono::ChVector<Real> ChAbs(const chrono::ChVector<Real> vect) {
        chrono::ChVector<Real> absVect;
        absVect.x() = std::abs(vect.x());
        absVect.y() = std::abs(vect.y());
        absVect.z() = std::abs(vect.z());
        return absVect;
    }


}  // end namespace frydom


#endif //FRYDOM_FRUTILS_H
