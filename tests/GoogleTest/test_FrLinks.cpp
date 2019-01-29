//
// Created by frongere on 25/01/19.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;


int main() {

    FrOffshoreSystem_ system;
    system.SetGravityAcceleration(0.1);

    // Body 1 definition (fixed body)
    auto body1 = system.NewBody();
    body1->SetBodyFixed(true);
    makeItBox(body1, 20, 10, 2, 1000);
    body1->SetCollide(false);
    body1->SetColor(Yellow);


    // Body 2 definition (linked body)
    auto body2 = system.NewBody();
    makeItBox(body2, 2, 2, 40, 2000);
    body2->SetColor(Black);
    body2->SetBodyFixed(true);



//    FrRotation_ rot(Direction(0, 1, 0), 90*DEG2RAD, NWU);



//    auto inertia = body2->GetInertiaTensor(NWU);
//    inertia.SetCOGPosition(Position(0, 0, 5), NWU);
//
//    body2->SetInertiaTensor(inertia);
//    body2->Rotate(FrRotation_(Direction(0, 0, 1), 20*DEG2RAD, NWU));




//
//    body2->Rotate(rot);
//    body2->TranslateInBody(Position(-5, 0, 0), NWU);
//
//    body2->TranslateInWorld(Position(10, 0, 0), NWU);







//    rot.RotX_RADIANS(10*DEG2RAD, NWU);
////    auto node1 = body1->NewNode(FrFrame_(Position(0, 0, 4), rot, NWU));
//    auto node1 = body1->NewNode(Position(0, 0, 0), rot, NWU);
//    auto node1 = body1->NewNode();
//    auto node2 = body2->NewNode(0, 0, 20, NWU);
//
//    auto prismatic = make_prismatic_link(node1, node2, &system);


    system.RunInViewer(0, 50, false);






    return 0;
}

