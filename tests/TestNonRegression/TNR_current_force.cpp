//
// Created by camille on 05/11/18.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

    // System

    FrOffshoreSystem_ system;

    // Ship

    auto ship = std::make_shared<FrBody_>();
    system.AddBody(ship);
    ship->SetAbsPosition(0., 0., 0., NWU);
    ship->SetCOGLocalPosition(0., 0. , 0.03, false, NWU);

    // Force

    auto current_force = std::make_shared<FrCurrentForce_>("../Ship_PolarCurrentCoeffs.yml");
    ship->AddExternalForce(current_force);

}