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


  linearFcn.WriteToGnuPlotFile(-3, 3, 0.01, "linear");

  auto linearRamp = FrLinearRampFunction(0, 1, 4, -1);

  auto absLinearRamp = abs(linearRamp);
  absLinearRamp.WriteToGnuPlotFile(-1, 5, 0.01, "absLinearRamp");


  auto newf = (3 * pow(x, 2) + 1) / sqrt(x);

  newf.WriteToGnuPlotFile(0.08, 5, 0.01, "newf");


  linearRamp.WriteToGnuPlotFile(-2, 6, 0.01, "ramp");

  auto saturate = saturate_both(linearRamp, -0.5, 0.5);

  saturate.WriteToGnuPlotFile(-3, 7, 0.01, "saturate");


  auto poly = FrPolynomialFunction(10.);
  poly.Add(2, 1);
  poly.Add(3, 2);
  poly.Add(4, 3);

  poly *= 3;

  poly.WriteToGnuPlotFile(-2, 2, 0.01, "poly");


  auto cosine = (1 + cos(2 * x)) / 2;


  cosine.WriteToGnuPlotFile(-10, 10, 0.01, "cosine");


  auto verif = pow(cos(x), 2);
  verif.WriteToGnuPlotFile(-10, 10, 0.01, "verif"); // FIXME : il y a des problemes avec les derivees !!!


  std::cout << (4 * atan(FrConstantFunction(1))).Get_y(0) << std::endl;


  auto tanhfcn = tanh(x);

  tanhfcn.WriteToGnuPlotFile(-10, 10, 0.01, "tanh");


  return 0;
}
