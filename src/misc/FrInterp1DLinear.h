//
// Created by frongere on 04/07/17.
//

#ifndef FRYDOM_FRINTERP1DLINEAR_H
#define FRYDOM_FRINTERP1DLINEAR_H

#include "FrInterp1d.h"

namespace frydom {

//    template <class Real=double>
//    class FrInterp1d;

    template <class Real=double>
    class FrInterp1DLinear : public FrInterp1d<Real> {


    public:
        virtual Real Eval(Real x) {};

        virtual std::vector<Real> Eval(const std::vector<Real> xvector) {};

    };



}  // end namespace frydom

#endif //FRYDOM_FRINTERP1DLINEAR_H
