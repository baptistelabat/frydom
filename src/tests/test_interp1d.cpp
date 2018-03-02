//
// Created by frongere on 04/07/17.
//

#include <cmath>

#include "../misc/FrInterp1DLinear.h"
#include "../misc/FrLinspace.h"

#define N 101

int main(int argc, char* argv[]) {
    // Declare two arrays to hold the coordinates and the initial data points
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

    std::cout << y0 << std::endl;

}