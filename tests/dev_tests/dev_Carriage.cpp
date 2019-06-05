//
// Created by lletourn on 04/06/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system;

    double tankLength = 140;
    double tankWidth = 5;
    double tankDepth = 3;

    auto tankWall = system.NewBody();
    tankWall->SetName("Wall");
    tankWall->SetFixedInWorld(true);
    makeItBox(tankWall, tankLength,1.,tankDepth,100.);
    tankWall->SetPosition(Position(0.,-0.5*tankWidth,-0.5*tankDepth),fc);

    auto wallNode = tankWall->NewNode();
    wallNode->SetPositionInBody(Position(0.,0.,2.), fc);
    wallNode->RotateAroundYInBody(90*DEG2RAD,fc);


    auto carriage = system.NewBody();
    carriage->SetName("Carriage");
    makeItBox(carriage, 0.1*tankWidth, tankWidth,1.,100.);
    carriage->SetPosition(Position(0.,0.5*tankWidth,2.), fc);

    auto carriageNode = carriage->NewNode();
    carriageNode->RotateAroundYInBody(90*DEG2RAD,fc);

    auto rail = make_prismatic_link(wallNode,carriageNode,&system);

    system.Initialize();
    system.DoAssembly();

    system.SetTimeStep(0.01);

    system.RunInViewer(0., 50, false);
}