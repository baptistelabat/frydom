//
// Created by frongere on 04/07/17.
//

#include <math.h>

#include "../misc/FrMathUtils.h"
#include "../misc/FrInterp1DLinear.h"
#include "../misc/FrLinspace.h"

#define N 101

int main(int argc, char* argv[]) {
    // Building the x coords as a shared pointer
    auto x = std::make_shared<std::vector<double>>(
            frydom::linspace(4*M_PI, M_PI, 1000, true)
    );

    // Building the data
    auto y = std::make_shared<std::vector<double>>();
    double val;
    for (int i = 0; i < x->size(); i++) {
        val = sin(x->at(i));
        y->push_back( val );
    }

    // Create the interpolation
    frydom::FrInterp1DLinear<double> interpolator;

    interpolator.Initialize(x, y);

    auto y0 = interpolator.Eval(5.3333);
    auto y1 = interpolator(5.3333);

//    std::cout << y0 << std::endl;
//    std::cout << interpolator(5.3333) << std::endl;

    assert(frydom::is_close(y0, y1));
    assert(frydom::is_close(y0, -0.8133409832926298));

    // Test for a vector of x coords
    auto x_interp = frydom::linspace(4*M_PI, M_PI, 5000, true);
    auto y_interp = interpolator(x_interp);

}