// ==========================================================================
// FRyDoM - frydom-ce.org
// 
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
// 
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
// 
// ==========================================================================

#include <chrono/core/ChVector.h>
#include <frydom/frydom.h>

using namespace frydom;
int main() {
    FrOffshoreSystem system;

    /// Set the current properties
    system.GetEnvironment()->GetCurrent()->Set(NORTH, 10, MS, NED, GOTO);
    system.GetEnvironment()->GetWind()->Set(NORTH, 10, MS, NED, GOTO);

    /// Create a Ship
    auto ship = std::make_shared<FrShip>();
    /// Ship initial position and orientation
    ship->SetPos(chrono::ChVector<>(0., 0., 0.));
    ship->SetNEDHeading(EAST);
    /// Ship initial velocity
    auto ship_velocity = ship->TransformDirectionLocalToParent(chrono::ChVector<>(25., 0., 0.));
    ship->SetPos_dt(ship_velocity);
    /// Set Rate of Turn
    ship->SetWvel_loc(chrono::ChVector<>(0,0,0));

    auto Node = ship->CreateNode(chrono::ChVector<>(10,0,0));

    /// Add the Ship to the Offshore System
    system.AddBody(ship);
    ship->Initialize();
    ship->Update();

    //auto currentRelativeVelocity = ship->GetCurrentRelativeVelocity(chrono::ChVector<>(10,0,0),NED,PARENT);
    auto currentRelativeVelocity = ship->GetCurrentRelativeVelocity(Node.get(),NED,PARENT);
    auto windRelativeVelocity = ship->GetWindRelativeVelocity(Node.get(),NED,PARENT);

    fmt::print("Current relative velocity : ({},{},{})\n",
               currentRelativeVelocity.x(),currentRelativeVelocity.y(),currentRelativeVelocity.z());
    fmt::print("Wind relative velocity    : ({},{},{})\n",
               windRelativeVelocity.x(),windRelativeVelocity.y(),windRelativeVelocity.z());


    return 0 ;
}
