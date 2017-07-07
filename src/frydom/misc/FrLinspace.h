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
    std::vector<Real> arange(Real start, Real stop, Real step=1) {
        assert (start < stop);
        assert (step > 0.);

        auto out = std::vector<Real>();

        Real d;
        Real new_val = start;
        while (true) {
            out.push_back(new_val);
            new_val += step;

            d = fabs(new_val - stop);

            if (is_close(new_val, stop)) break;
            if (new_val >= stop) break;
        }
        return out;
    }


    template<class Real=double>
    std::vector<Real> linspace(Real start, Real stop, unsigned int num) {
        assert(num > 1);
        assert(start <= stop);

        Real step = (stop-start) / (num-1);
        return arange(start, stop + step / 2, step);

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

}