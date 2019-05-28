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

//    system.GetWorldBody()->AddSphereShape(10.);

    // Body1 definition
    auto body1 = system.NewBody();
    body1->SetName("1");
    makeItBox(body1, 20, 10, 2, 1000);
    body1->AllowCollision(false);
    body1->SetColor(MediumVioletRed);

    body1->SetPosition(Position(5.,5.,5.),NWU);

    // Apply a random translation and rotation to the body to check if the assembly is done correctly
//    body1->TranslateInWorld(9.,-6.,25, NWU);
//    Direction randomDir(1.,5.,-9.); randomDir.normalize();
//    FrRotation randomRotation(randomDir,0.2954,NWU);
//    body1->Rotate(randomRotation);

    // Revolute link between body1 and world
    auto node1 = body1->NewNode();
    node1->TranslateInBody(10, 5, 0, NWU);
    node1->ShowAsset(true);
    node1->GetAsset()->SetSize(10);
//    node1->RotateAroundXInBody(90*DEG2RAD, NWU);
//    node1->RotateAroundZInBody(90*DEG2RAD, NWU);


    auto nodeWorld = system.GetWorldBody()->NewNode();
    nodeWorld->ShowAsset(true);
    nodeWorld->GetAsset()->SetSize(10);
//    nodeWorld->TranslateInWorld(10, 0, 0, NWU);
//    nodeWorld->RotateAroundXInBody(45*DEG2RAD, NWU);
//    nodeWorld->RotateAroundZInBody(90*DEG2RAD, NWU);

    auto point1 = std::make_shared<FrPoint>(node1);
    auto pointWorld = std::make_shared<FrPoint>(nodeWorld);

    auto axis1 = std::make_shared<FrAxis>(node1,XAXIS);
    auto axisWorld = std::make_shared<FrAxis>(nodeWorld,XAXIS);

    auto plane1 = std::make_shared<FrPlane>(node1,YAXIS);
    auto planeWorld = std::make_shared<FrPlane>(nodeWorld,ZAXIS);

    auto constraint = make_constraint_distance_to_axis(axisWorld, point1, &system, false, 10.);

    system.Initialize();
    system.GetChronoSystem()->DoFullAssembly();

    // Run the simulation (or visualize the assembly)
    system.SetTimeStep(0.01);
    system.RunInViewer(0, 50, true);
//    system.Visualize(50, false);

    return 0;
}