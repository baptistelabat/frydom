//
// Created by frongere on 06/02/19.
//

#include "frydom/frydom.h"


using namespace frydom;

int main() {

    // Ramp 1
    auto ramp1 = FrRampFunction_();
    ramp1.SetByTwoPoints(0, 0, 3, 3);
//    ramp1->SetY0(0.);
//    ramp1->SetSlope(0.5.);
//    ramp1->SetXWindow(0., 1.);
//
//    std::cout << ramp1->Get_y(-1.) << std::endl; // 0.
//    std::cout << ramp1->Get_y(0.) << std::endl;  // 0.
//    std::cout << ramp1->Get_y(0.5) << std::endl; // 0.5
//    std::cout << ramp1->Get_y(1.) << std::endl;  // 1.
//    std::cout << ramp1->Get_y(1.5) << std::endl; // 1.

    std::cout << std::endl;

    // Ramp2
    auto ramp2 = FrRampFunction_();
    ramp2.SetY0(1.);
    ramp2.SetSlope(-1.);
    ramp2.SetXWindow(0., 5.);
    ramp2.SetByTwoPoints(0, 0, 3, 1);
    ramp2.SetXOffset(4.);
    ramp2.WriteToGnuPlotFile(-2, 10, 0.01);


//    std::cout << ramp2->Get_y(-1.) << std::endl; // 1.
//    std::cout << ramp2->Get_y(0.) << std::endl;  // 1.
//    std::cout << ramp2->Get_y(0.5) << std::endl; // 0.5
//    std::cout << ramp2->Get_y(1.) << std::endl;  // 0.
//    std::cout << ramp2->Get_y(1.5) << std::endl; // 0.
//
//    std::cout << std::endl;
//
//    // Tester maintenant les operations sur les fonctions
//    auto add_func = *ramp1 + ramp2;
//
//    std::cout << add_func->Get_y(-1.) << std::endl; // 1.
//    std::cout << add_func->Get_y(0.) << std::endl;  // 1.
//    std::cout << add_func->Get_y(0.5) << std::endl; // 0.5
//    std::cout << add_func->Get_y(1.) << std::endl;  // 0.
//    std::cout << add_func->Get_y(1.5) << std::endl; // 0.
//
//    std::cout << std::endl;
//
//    // Tester maintenant les operations sur les fonctions
//    auto sub_func = *ramp1 - ramp1;
//
//    std::cout << sub_func->Get_y(-1.) << std::endl; // 1.
//    std::cout << sub_func->Get_y(0.) << std::endl;  // 1.
//    std::cout << sub_func->Get_y(0.5) << std::endl; // 0.5
//    std::cout << sub_func->Get_y(1.) << std::endl;  // 0.
//    std::cout << sub_func->Get_y(1.5) << std::endl; // 0.





    // Test avec plusieurs operations en meme temps
//    auto complex_func = *(*(*(*ramp1 + ramp1)) / 2. * 2) << ramp1;


    auto sinFcn = FrSinFunction();
    sinFcn.SetPeriod(2);
    sinFcn.SetYOffset(0.5);
    sinFcn.SetPhase(MU_PI);
    sinFcn.WriteToGnuPlotFile(0, 20, 0.01, "sinFunction");



    auto thRamp = FrTanhRampFunction();
    thRamp.SetByTwoPoints(0, 1, 1, 1);
//    thRamp.SetXOffset(1.); // FIXME -> ne fonctionne pas !!
    thRamp.WriteToGnuPlotFile(-2., 4., 0.01, "thRamp");


//    auto op = 1. + -2 * ramp1 * 0.5 + 4;
//    auto op = 1 + (1 * ramp1) + 3; // Voir si on peut faire fonctionner ça !!
    auto op =  ((1 * ramp1) - 10);
    FrSinFunction sin;
//    auto op2 =  1 + (sin << thRamp);

    op2.WriteToGnuPlotFile(-2, 4, 0.01, "op");

    return 0;
}
