//
// Created by frongere on 13/07/17.
//

#include "frydom/core/FrCore.h"
#include "frydom/environment/FrEnvironment.h"

using namespace frydom;
using namespace environment;

int main(int argc, char* argv[]) {

    // The system
    FrOffshoreSystem system;

    // The current
    auto current_field = std::make_unique<FrCurrent>(EAST, 5, KNOT);
    // TODO: changer pour faire des move !!
    system.setCurrent(current_field.release());

    // A ship
    auto ship = std::make_shared<FrShip>();
//    ship.SetHydroMesh("");
    system.AddBody(ship);
    // TODO: voir plus tard a ajouter les asset etc...

    // Building current force
    std::string filename("../src/frydom/tests/data/PolarCurrentCoeffs.yml");
    auto current_force = std::make_shared<FrCurrentForce>(filename);
    ship->AddForce(current_force);

    current_force->UpdateState();

    return 0;
}