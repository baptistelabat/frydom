//
// Created by frongere on 06/02/19.
//

#include "frydom/frydom.h"


using namespace frydom;

int main() {

    // Ramp 1
    auto ramp1 = std::make_shared<FrRamp_>();
    ramp1->SetY0(0.);
    ramp1->SetSlope(1.);
    ramp1->SetXLimits(0., 10.);

    std::cout << ramp1->Get_y(-1.) << std::endl; // 0.
    std::cout << ramp1->Get_y(0.) << std::endl;  // 0.
    std::cout << ramp1->Get_y(0.5) << std::endl; // 0.5
    std::cout << ramp1->Get_y(1.) << std::endl;  // 1.
    std::cout << ramp1->Get_y(1.5) << std::endl; // 1.

    std::cout << std::endl;

    // Ramp2
    auto ramp2 = std::make_shared<FrRamp_>();
    ramp2->SetY0(1.);
    ramp2->SetSlope(-1.);
    ramp2->SetXLimits(0., 1.);

    std::cout << ramp2->Get_y(-1.) << std::endl; // 1.
    std::cout << ramp2->Get_y(0.) << std::endl;  // 1.
    std::cout << ramp2->Get_y(0.5) << std::endl; // 0.5
    std::cout << ramp2->Get_y(1.) << std::endl;  // 0.
    std::cout << ramp2->Get_y(1.5) << std::endl; // 0.

    std::cout << ramp2->Get_y(11) << std::endl; // 0.


    // Tester maintenant les operations sur les fonctions
    auto add_func = *ramp1 + ramp2;


    return 0;
}
