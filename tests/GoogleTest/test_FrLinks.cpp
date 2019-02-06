//
// Created by frongere on 25/01/19.
//

#include "frydom/frydom.h"
//#include "gtest/gtest.h"

using namespace frydom;

int main() {

    FrOffshoreSystem_ system;
    system.GetEnvironment()->ShowFreeSurface(false);

    // Body1 definition
    auto body1 = system.NewBody();
    makeItBox(body1, 20, 10, 2, 1000);
    body1->AllowCollision(false);
    body1->SetColor(MediumVioletRed);

    auto node1 = body1->NewNode();
    node1->TranslateInBody(-10, 0, 0, NWU);
    node1->RotateAroundXInBody(90*DEG2RAD, NWU);

    auto nodeWorld = system.GetWorldBody()->NewNode();
    nodeWorld->TranslateInWorld(-10, 0, 0, NWU);
    nodeWorld->RotateAroundXInBody(90*DEG2RAD, NWU);

    auto rev1 = make_revolute_link(node1, nodeWorld, &system);

    // Body 2 definition (linked body)
    auto body2 = system.NewBody();
    makeItBox(body2, 2, 2, 40, 2000);
    body2->SetColor(Black);
    body2->TranslateInWorld(10, 5, 0, NWU);

    auto m1 = body1->NewNode();
    m1->TranslateInBody(10, 5, -1, NWU);

    auto m2 = body2->NewNode();
    m2->TranslateInBody(-1, -1, -20, NWU);

    auto prismaticLink = make_prismatic_link(m1, m2, &system);
    prismaticLink->SetSpringDamper(2e3, 1e3);
    prismaticLink->SetRestLength(-5);

    // Body 3 definition
    auto body3 = system.NewBody();
    makeItBox(body3, 2, 2, 6, 500);
    body3->AllowCollision(false);
    body3->SetColor(Red);
    body3->TranslateInWorld(-10, 5, -1, NWU);

    auto m3 = body1->NewNode();
    m3->TranslateInBody(-10, 5, -1, NWU);

    auto m4 = body3->NewNode();

    auto revoluteLink = make_revolute_link(m3, m4, &system);
    revoluteLink->SetSpringDamper(1e4, 1e1);
    revoluteLink->SetRestAngle(180*DEG2RAD);

    system.SetTimeStep(0.01);
    system.RunInViewer(0, 50, false);

    return 0;
}

