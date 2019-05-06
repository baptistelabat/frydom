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
//#include "gtest/gtest.h"

using namespace frydom;

int main() {

    FrOffshoreSystem system;
    system.GetEnvironment()->ShowFreeSurface(false);

    // Body1 definition
    auto body1 = system.NewBody();
    body1->SetName("1");
    makeItBox(body1, 20, 10, 2, 1000);
    body1->AllowCollision(false);
    body1->SetColor(MediumVioletRed);

    // Apply a random translation and rotation to the body to check if the assembly is done correctly
//    body1->TranslateInWorld(9.,-6.,25, NWU);
//    Direction randomDir(1.,5.,-9.); randomDir.normalize();
//    FrRotation randomRotation(randomDir,0.2954,NWU);
//    body1->Rotate(randomRotation);

    // Revolute link between body1 and world
    auto node1 = body1->NewNode();
    node1->TranslateInBody(-10, 0, 0, NWU);
    node1->RotateAroundXInBody(90*DEG2RAD, NWU);

    auto nodeWorld = system.GetWorldBody()->NewNode();
    nodeWorld->TranslateInWorld(-10, 0, 0, NWU);
    nodeWorld->RotateAroundXInBody(90*DEG2RAD, NWU);

    auto rev1 = make_revolute_link(node1, nodeWorld, &system);

    auto motor1 = rev1->Motorize(POSITION);

    auto t = new_var("t");
    motor1->SetMotorFunction(MU_PI * sin(t));
//    motor1->SetMotorFunction(FrConstantFunction(MU_PI_2));


    // Body 2 definition (linked body)
    auto body2 = system.NewBody();
    body2->SetName("2");
    makeItBox(body2, 2, 2, 40, 2000);
    body2->SetColor(Black);
//    body2->TranslateInWorld(10, 5, 0, NWU);

//    // Apply a random translation and rotation to the body to check if the assembly is done correctly
//    body2->TranslateInWorld(-2.,7.,8, NWU);
//    randomDir = Direction(6.,-8.,45); randomDir.normalize();
//    randomRotation = FrRotation(randomDir,0.9687,NWU);
//    body2->Rotate(randomRotation);
//
    // Prismatic link between body1 and body2
    auto m1 = body1->NewNode();
    m1->TranslateInBody(10, 5, -1, NWU);

    auto m2 = body2->NewNode();
    m2->TranslateInBody(-1, -1, -20, NWU);

    auto prismaticLink = make_prismatic_link(m1, m2, &system);
//    prismaticLink->SetSpringDamper(2e3, 1e3);
//    prismaticLink->SetRestLength(-5);

    auto motor2 = prismaticLink->Motorize(VELOCITY);
    motor2->SetMotorFunction(10*sin(t));

//    system.RemoveLink(prismaticLink);


//    // Body 3 definition
//    auto body3 = system.NewBody();
//    body3->SetName("3");
//    makeItBox(body3, 2, 2, 6, 500);
//    body3->AllowCollision(false);
//    body3->SetColor(Red);
////    body3->TranslateInWorld(-10, 5, -1, NWU);
//
//    // Apply a random translation and rotation to the body to check if the assembly is done correctly
//    body3->TranslateInWorld(0.,1.,-9, NWU);
//    randomDir = Direction(26.,98.,5); randomDir.normalize();
//    randomRotation = FrRotation(randomDir,-0.57,NWU);
//    body3->Rotate(randomRotation);
//
//    // Revolute link between body1 and body3
//    auto m3 = body1->NewNode();
//    m3->TranslateInBody(-10, 5, -1, NWU);
//
//    auto m4 = body3->NewNode();
//
//    auto revoluteLink = make_revolute_link(m3, m4, &system);
//    revoluteLink->SetSpringDamper(1e4, 1e1);
//    revoluteLink->SetRestAngle(180*DEG2RAD);

    // Run the simulation (or visualize the assembly)
    system.SetTimeStep(0.01);
    system.RunInViewer(0, 50, false);
//    system.Visualize(50, false);

    return 0;
}

