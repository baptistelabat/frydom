//
// Created by frongere on 04/07/17.
//

#include <vector>
#include <iostream>
#include <assert.h>
#include <limits>
#include "math.h"
#include "FrMathUtils.h"


namespace frydom {

    template<class Real=double>
    std::vector<Real> linspace(Real start, Real stop, unsigned int num) {
        assert(num > 1);
        assert(start <= stop);

        Real step = (stop-start) / (num-1);

        std::vector<Real> out(num);
        for (unsigned long i=0; i<num; ++i) {
            out[i] = start + i*step;
        }

        return out;
    }

    template <class Real=double>
    std::vector<Real> logspace(Real start, Real stop, unsigned int num, Real base=10.) {
        assert(num > 1);
        assert(start <= stop);

        auto out = linspace(start, stop, num);
        for (unsigned int i = 0; i < out.size(); i++) {
            out.at(i) = pow(base, out.at(i));
        }
        return out;
    }

    template<class Real=double>
    std::vector<Real> arange(Real start, Real stop, Real step=1) {
        assert (start < stop);
        assert (step > 0.);

        auto num = floor((stop-start)/step);
        auto final_stop = num * step;

        return linspace(start, final_stop, (uint)num);
    }

}