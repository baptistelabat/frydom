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
    auto current_field = std::make_unique<FrCurrent>(NORTH_EAST, 5, KNOT, NED, COMEFROM);
    // TODO: changer pour faire des move !!
    system.setCurrent(current_field.release());

    // A ship
    auto ship = std::make_shared<FrShip>();
    ship->SetNEDHeading(45+180, DEG);

    auto ship_velocity = ship->TransformDirectionLocalToParent(chrono::ChVector<>(1, 0, 0));
    ship->SetPos_dt(ship_velocity);  // TODO: ajouter des fonctions

//    ship.SetHydroMesh("");
    system.AddBody(ship);
    // TODO: voir plus tard a ajouter les asset etc...

    // Building current force
    std::string filename("../src/frydom/tests/data/PolarCurrentCoeffs.yml");
    auto current_force = std::make_shared<FrCurrentForce>(filename);
    ship->AddForce(current_force);


    system.Update();

//    ship->Update(true);


    return 0;
}