//
// Created by frongere on 29/06/17.
//

#ifndef FRYDOM_FRUTILS_H
#define FRYDOM_FRUTILS_H

#include <cmath>

// TODO: rename to FrMathUtils.h

namespace frydom {

    template <class Real=double>
    inline bool is_close(const Real a, const Real b, const Real rtol=1e-5, const Real atol=1e-8) {
        return (fabs(a-b) <= (atol + rtol*fabs(b)));
    }

    template <class Real=double>
    inline bool is_unit_vector(chrono::ChVector<Real> vect) {
        return is_close(vect.Length2(), 1.);
    }


}  // end namespace frydom


#endif //FRYDOM_FRUTILS_H
