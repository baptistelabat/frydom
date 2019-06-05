//
// Created by lletourn on 04/06/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system;

    // --------------------------------------------------
    // Ship
    // --------------------------------------------------

    Position shipConnection;

    auto ship = system.NewBody();
    ship->SetName("Ship");
    ship->AddMeshAsset("DTMB5512.obj");
    ship->SetColor(Green);

//    ship->SetPosition(Position(-450.,0,0), NWU);

    // Inertia
    double mass = 5e7; double Ixx = 1e8; double Iyy = 1e9; double Izz = 1e9;
    ship->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., FrFrame(), NWU));

    // Hydrodynamic Database
//    auto hdb = make_hydrodynamic_database("Ship.h5");

    auto eqFrame = std::make_shared<FrEquilibriumFrame>(ship.get());
    system.AddPhysicsItem(eqFrame);

//    hdb->Map(0,ship.get(),eqFrame);

    auto hydrostaticForce = make_linear_hydrostatic_force(eqFrame,ship,"DTMB5512.obj",FrFrame());

    shipConnection = ship->GetCOGPositionInWorld(fc);

    auto shipNode = ship->NewNode();
    shipNode->SetPositionInWorld(shipConnection,fc);
    shipNode->RotateAroundYInBody(90*DEG2RAD,fc);
    shipNode->RotateAroundXInBody(90*DEG2RAD,fc);


    // --------------------------------------------------
    // Carriage
    // --------------------------------------------------

    double tankLength = 140;
    double tankWidth = 5;
    double tankDepth = 3;

    auto Seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
    Seabed->GetSeabedGridAsset()->SetGrid(-0.05*tankLength, 0.95*tankLength, 2, -0.5*tankWidth, 0.5*tankWidth, 2);
    Seabed->SetBathymetry(-tankDepth,fc);

    system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-0.05*tankLength, 0.95*tankLength, 2, -0.5*tankWidth, 0.5*tankWidth, 2);


    auto tankWall = system.NewBody();
    tankWall->SetName("Wall");
    tankWall->SetFixedInWorld(true);
    makeItBox(tankWall, tankLength,1.,1.25*tankDepth,100.);
    Position tankWallPosition = shipConnection; tankWallPosition.GetZ() = 0.;
    tankWallPosition -= Position(-0.45*tankLength,0.5*tankWidth,0.5*tankDepth);
    tankWall->SetPosition(tankWallPosition,fc);

    auto wallNode = tankWall->NewNode();
    wallNode->SetPositionInBody(Position(0.45*tankLength,0.,0.75*tankDepth), fc);
    wallNode->RotateAroundYInBody(90*DEG2RAD,fc);


    auto carriage = system.NewBody();
    carriage->SetName("Carriage");
    makeItBox(carriage, 0.1*tankWidth, tankWidth,0.25,100.);
    Position carriagePosition = shipConnection; carriagePosition.GetZ() = 0.;
//    carriagePosition += Position(0.,0.,2.);
    carriage->SetPosition(carriagePosition, fc);

    auto carriageToWallNode = carriage->NewNode();
    carriageToWallNode->SetPositionInBody(Position(0.,-0.5*tankWidth,0.), fc);
    carriageToWallNode->RotateAroundYInBody(90*DEG2RAD,fc);

    carriage->SetPositionOfBodyPoint(carriageToWallNode->GetNodePositionInBody(fc),wallNode->GetPositionInWorld(fc),fc);

    auto carriageToShipNode = carriage->NewNode();
    carriageToShipNode->SetPositionInWorld(shipConnection, fc);
    carriageToShipNode->RotateAroundYInBody(90*DEG2RAD,fc);
    carriageToShipNode->RotateAroundXInBody(90*DEG2RAD,fc);

    auto rail = make_prismatic_link(wallNode,carriageToWallNode,&system);

    auto linkToShip = make_prismatic_revolute_link(carriageToShipNode, shipNode, &system);


//    system.Initialize();
//    system.DoAssembly();

    system.SetTimeStep(0.01);

    system.RunInViewer(0., 10, false);
}