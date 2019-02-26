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

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    /** This demo presents basic features of FRyDoM, with EasyBody (box and spheres). The makeIt* functions set-up the
     * collision box, corresponding asset and inertia tensor. We simulate the free fall of several balls with
     * an initial velocity on a floor, with collisions between the balls.
     */

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem system;

    // Hide the free surface and seabed visual assets.
    system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);
    system.GetEnvironment()->GetOcean()->ShowSeabed(false);

    // ------------------ Floor ------------------ //

    // Create the floor box (with a collision box already defined from makeItBox)
    auto floorBox = system.NewBody();
    floorBox->SetName("Floor");
    makeItBox(floorBox,100.,100.,2.,1000.);
    floorBox->SetColor(Green);
    floorBox->SetPosition(Position(0.,0.,0.),fc);
    floorBox->SetFixedInWorld(true);

    // ------------------ Balls ------------------ //

    // Set the number of balls you want, their radius and density.
    int n_balls = 10;
    double dtheta = 2*M_PI/n_balls;
    double radius = 1.;
    double density = 250;

    // Create as many balls as specified
    for (int ib=0;ib<n_balls;++ib){
        // A new ball is created in the offshore system
        auto ball = system.NewBody();

        // Give it a name
        std::stringstream concatenation;
        concatenation<<"Ball"<<ib;
        ball->SetName(concatenation.str().c_str());

        // Make it a sphere, with a collision box, spheric asset and inertia tensor automatically calculated
        makeItSphere(ball,radius,density);

        // Set the color
        ball->SetColor(Red);

        // Set the initial position and velocity
        ball->SetPosition(Position(100*cos(ib * dtheta),100*sin(ib * dtheta),30.),fc);
        ball->SetVelocityInWorldNoRotation(Velocity(-20.*cos(ib * dtheta),-20.*sin(ib * dtheta),20.),fc);
    }

    // ------------------ Run ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.04);

    // Don't forget to initialize the offshore system : it will initialize every physical objects and environmental
    // components it contains.
    system.Initialize();

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 15) and the distance from the camera to the objectif (75m).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(15, 75, false);


}
