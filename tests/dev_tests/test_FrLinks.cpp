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

#include <chrono/physics/ChBodyEasy.h>
#include "frydom/frydom.h"
//#include "gtest/gtest.h"

using namespace frydom;

int main() {

    FrOffshoreSystem system;
    system.GetEnvironment()->ShowFreeSurface(false);
//    system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(-50,NWU);
    system.GetEnvironment()->ShowSeabed(false);
    system.SetName("Links");

    auto worldBody = system.GetWorldBody();
//    system.GetWorldBody()->AddSphereShape(10.);

    // Body1 definition
    auto body = system.NewBody();
    body->SetName("1");
    makeItBox(body, 20, 10, 2, 1000);
    body->AllowCollision(false);
    body->SetColor(MediumVioletRed);

    auto leftNode = body->NewNode();
    leftNode->SetPositionInBody(Position(0,-5,0),NWU);
    leftNode->RotateAroundYInBody(95*DEG2RAD,NWU);

    auto rightNode = body->NewNode();
    rightNode->SetPositionInBody(Position(0,5,0),NWU);
    rightNode->RotateAroundYInBody(95*DEG2RAD,NWU);

    auto leftWBNode = worldBody->NewNode();
    leftWBNode->SetPositionInBody(Position(0,-5,0),NWU);
    leftWBNode->RotateAroundYInBody(95*DEG2RAD,NWU);

    auto rightWBNode = worldBody->NewNode();
    rightWBNode->SetPositionInBody(Position(0,5,0),NWU);
    rightWBNode->RotateAroundYInBody(95*DEG2RAD,NWU);

    auto leftLink = make_prismatic_link(leftWBNode, leftNode, &system);
    leftLink->SetName("left");

//    auto leftMotor = leftLink->Motorize(POSITION);
//    leftMotor->SetMotorFunction(FrConstantFunction(0.));

    auto rightLink = make_prismatic_link(rightWBNode, rightNode, &system);
    rightLink->SetName("right");

    system.SetSolver(FrOffshoreSystem::SOLVER::BARZILAIBORWEIN);
    system.SetSolverVerbose(true);
    system.SetSolverWarmStarting(true);
    system.SetSolverMaxIterSpeed(10);

    system.Initialize();
    system.GetChronoSystem()->DoFullAssembly();

    // Run the simulation (or visualize the assembly)
    system.SetTimeStep(0.01);
    system.RunInViewer(0, 50, true);
//    system.Visualize(50, false);

    return 0;
}