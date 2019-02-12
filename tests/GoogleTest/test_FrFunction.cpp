//
// Created by frongere on 06/02/19.
//

#include "frydom/frydom.h"


using namespace frydom;

int main() {

    // X function
    auto x = new_var("x");



//    auto linearFcn = clamp_before(clamp_after(3*pow(x, 2)+2, 1), -1);
    auto linearFcn = FrLinearFunction(2, 3) << pow(x, 2);

//    std::cout << linearFcn.Get_y(2.) << std::endl;


    linearFcn.WriteToGnuPlotFile(-3, 3, 0.01, "linear");

    auto linearRamp = FrLinearRampFunction_(0, 1, 4, -1) << pow(x, 2);

    linearRamp.WriteToGnuPlotFile(-2, 6, 0.01, "ramp");

    auto saturate = saturate_both(linearRamp, -0.5, 0.5);

    saturate.WriteToGnuPlotFile(-3, 7, 0.01, "saturate");



    auto poly = FrPolynomialFunction(10.);
    poly.Add(2, 1);
    poly.Add(3, 2);
    poly.Add(4, 3);

    poly *= 3;

    poly.WriteToGnuPlotFile(-2, 2, 0.01, "poly");



//    // Ramp 1
//    auto ramp1 = FrLinearRampFunction_();
//    ramp1.SetByTwoPoints(0, 0, 3, 3);
//
//
//    auto newFcn1 = + ramp1;
//    auto newFcn2 = - ramp1;
//
//    ramp1.WriteToGnuPlotFile(-2, 5, 0.01, "ramp1");
//    newFcn1.WriteToGnuPlotFile(-2, 5, 0.01, "newFcn1");
//    newFcn2.WriteToGnuPlotFile(-2, 5, 0.01, "newFcn2");
//
//    (newFcn1 - newFcn2).WriteToGnuPlotFile(-2, 5, 0.01, "sum");
//    (newFcn1 << newFcn2).WriteToGnuPlotFile(-2, 5, 0.01, "sum");
//
//    std::cout << (newFcn1 - newFcn2).GetRepr() << std::endl;
//
//
//    // Test monomial
//    auto monomial = FrPowFunction(2);
//    monomial.WriteToGnuPlotFile(-10, 10, 0.01, "pow2");
//
//
//
////    // Testing polynomial
////    auto poly = FrPolynomialFunction();
////    poly.Add(3, 2);
////    poly.Add(6, 1);
////    poly.Add(1, 0);
////
////    poly.WriteToGnuPlotFile(-10, 10, 0.01, "poly");



    return 0;
}
