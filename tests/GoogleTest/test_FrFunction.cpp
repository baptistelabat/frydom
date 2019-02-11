//
// Created by frongere on 06/02/19.
//

#include "frydom/frydom.h"


using namespace frydom;

int main() {

    // Ramp 1
    auto ramp1 = FrRampFunction_();
    ramp1.SetByTwoPoints(0, 0, 3, 3);


    auto newFcn1 = + ramp1;
    auto newFcn2 = - ramp1;

    ramp1.WriteToGnuPlotFile(-2, 5, 0.01, "ramp1");
    newFcn1.WriteToGnuPlotFile(-2, 5, 0.01, "newFcn1");
    newFcn2.WriteToGnuPlotFile(-2, 5, 0.01, "newFcn2");

    (newFcn1 - newFcn2).WriteToGnuPlotFile(-2, 5, 0.01, "sum");
    (newFcn1 << newFcn2).WriteToGnuPlotFile(-2, 5, 0.01, "sum");

    std::cout << (newFcn1 - newFcn2).GetRepr() << std::endl;


    // Testing polynomial
    auto poly = FrPolynomialFunction();
    poly.Add(3, 2);
    poly.Add(6, 1);
    poly.Add(1, 0);

    poly.WriteToGnuPlotFile(-10, 10, 0.01, "poly");



    return 0;
}
