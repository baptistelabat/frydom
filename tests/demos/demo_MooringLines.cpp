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

    /**
     * This demo presents the definition of dynamic cable. A distinction is to be made between cables taut or not at
     * initialisation.
     *
     * In the case of non taut cable, the dynamic cable position is initialized using a non elastic catenary model.
     * If the elasticity of the cable is not too large, the dynamic cable position is close to its static equilibrium.
     *
     * For taut cable, the dynamic cable is initialized unstrained and then strained either during a static analysis
     * or the dynamic simulation. This means that the cable ending node is not located at its specified location.
     *
     * The hinges (joints) to connect the cable to bodies can be defined as completely constrained (no degrees of freedom)
     * or spherical (degree of freedom in rotation not constrained). By default, they are defined as spherical.
     *
     * Be careful on the definition of the frame nodes, on the bodies, in the case of constrained hinges. The x axis of
     * the frame node must corresponds to the direction of the cable you want to get at this cable end.
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

    // Create the nodes from the world body (which is fixed)
    auto Node1 = system.GetWorldBody()->NewNode();
    Node1->SetPositionInBody(Position(-10., 0., 0.), NWU);
    auto Node2 = system.GetWorldBody()->NewNode();
    Node2->SetPositionInBody(Position(10., 0., 0.), NWU);

    auto Node3 = system.GetWorldBody()->NewNode();
    Node3->SetPositionInBody(Position(-10., 1., 0.), NWU);
    auto Node4 = system.GetWorldBody()->NewNode();
    Node4->SetPositionInBody(Position(10., 1., 0.), NWU);

    // Line properties : you can change the unstretched length to 18m to test a taut line.
    double unstretchedLength = 18.;                         //  unstretched length, in m
    auto cableProp = make_cable_properties();
    cableProp->SetDiameter(0.1);
    cableProp->SetEA(1.5708e7);
    cableProp->SetLinearDensity(616.538);
    double rayleighDamping = 0.;                            //  Rayleigh Damping
    unsigned int nbElements = 50;                           //  number of elements

    // Dynamic cable
    auto DynamicCable = make_dynamic_cable(Node1, Node2, &system, cableProp, unstretchedLength, rayleighDamping, nbElements);

    // To test with constrained hinges, uncomment the following lines.
//    DynamicCable->SetStartingHingeType(FrDynamicCable::CONSTRAINED);
//    DynamicCable->SetEndingHingeType(FrDynamicCable::CONSTRAINED);
//    Node2->RotateAroundZInBody(MU_PI, NWU); // need to set the frame node orientation correctly.

    // Catenary line for comparison purpose
    auto CatenaryLine = make_catenary_line(Node3, Node4, &system, cableProp, true, unstretchedLength, AIR);

    // Change solver settings, for dynamic cable modeling
    system.SetSolverWarmStarting(true);
    system.SetSolverMaxIterSpeed(200);
    system.SetSolverMaxIterStab(200);
    system.SetSolverForceTolerance(1e-13);

    // Set the dynamic simulation time step
    system.SetTimeStep(0.01);

    // Run the simulation
    system.RunInViewer(0.,20);



}