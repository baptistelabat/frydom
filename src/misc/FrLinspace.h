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
    std::vector<Real> arange(Real stop, Real step = 1, Real start = 0) {
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
    std::vector<Real> linspace(Real stop, Real start, unsigned int num = 50, bool endpoint = true) {

        assert(num >= 0);
        assert(start <= stop);

        Real div;
        if (endpoint) {
            div = num - 1;
        } else {
            div = num;
        }

        auto delta = stop - start;

        Real step;
        if (num > 1) {
            step = delta / div;
            if (endpoint) {
                return arange(stop + step / 2, step, start);
            } else {
                return arange(stop, step, start);
            }
        }

    }
}