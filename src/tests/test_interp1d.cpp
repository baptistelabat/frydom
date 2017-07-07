//
// Created by frongere on 04/07/17.
//

#include <math.h>

#include "misc/FrMathUtils.h"
#include "misc/FrInterp1dLinear.h"
#include "misc/FrLinspace.h"

#define N 10001

int main(int argc, char* argv[]) {
    // Building the x coords as a shared pointer
    auto x = std::make_shared<std::vector<double>>(
            frydom::linspace(M_PI, 4*M_PI, N-1)
    );

    // Building the data
    auto y = std::make_shared<std::vector<double>>();
    double val;
    for (unsigned long i = 0; i < x->size(); i++) {
        val = sin(x->at(i));
        y->push_back( val );
    }

    // Create the interpolation
    frydom::FrInterp1dLinear<double> interpolator;

    interpolator.Initialize(x, y);

    // Test of the Eval method for one scalar
    auto y0 = interpolator.Eval(5.3333);
    // Test of the call operator for one scalar
    auto y1 = interpolator(5.3333);

    assert(frydom::is_close(y0, y1));
    assert(frydom::is_close(y0, -0.8133409832926298));

    // Test for a vector of x coords
    auto x_interp = frydom::linspace(M_PI, 4*M_PI, 1000*N);
    // Using only the overloaded call operator for vector values
    auto y_interp = interpolator(x_interp);

}