//
// Created by frongere on 26/07/17.
//

#ifndef FRYDOM_FRUTILS_H
#define FRYDOM_FRUTILS_H

#include <iostream>
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

}  // end namespace frydom


#endif //FRYDOM_FRUTILS_H
