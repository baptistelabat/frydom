//
// Created by lletourn on 04/06/19.
//

#include "frydom/frydom.h"

using namespace frydom;

FrLinearActuator* make_carriage(FrOffshoreSystem* system, const std::shared_ptr<FrNode>& shipNode){

    FRAME_CONVENTION fc = NWU;

    auto mass = shipNode->GetBody()->GetInertiaTensor(fc).GetMass();

    double tankLength = 140;
    double tankWidth = 5;
    double tankDepth = 3;


    // --------------------------------------------------
    // Seabed and Free-surface grid definitions
    // --------------------------------------------------
    auto Seabed = system->GetEnvironment()->GetOcean()->GetSeabed();
    Seabed->GetSeabedGridAsset()->SetGrid(
            -0.05*tankLength, 0.95*tankLength, 0.01*tankLength,
            -0.5*tankWidth, 0.5*tankWidth, 0.01*tankWidth);
    Seabed->SetBathymetry(-tankDepth,fc);

    auto FreeSurface = system->GetEnvironment()->GetOcean()->GetFreeSurface();
    FreeSurface->GetFreeSurfaceGridAsset()->SetGrid(
            -0.05*tankLength, 0.95*tankLength, 0.01*tankLength,
            -0.5*tankWidth, 0.5*tankWidth, 0.01*tankWidth);
    FreeSurface->GetFreeSurfaceGridAsset()->UpdateAssetON();
    FreeSurface->GetFreeSurfaceGridAsset()->SetUpdateStep(10);

    // --------------------------------------------------
    // Wall definition
    // --------------------------------------------------

    auto tankWall = system->NewBody();
    tankWall->SetName("Wall");
    tankWall->SetFixedInWorld(true);
    makeItBox(tankWall, tankLength, 0.1*tankWidth, 1.25*tankDepth, mass);

    Position tankWallPosition = shipNode->GetPositionInWorld(fc); tankWallPosition.GetZ() = 0.;
    tankWallPosition -= Position(-0.45*tankLength,0.55*tankWidth,0.375*tankDepth);
    tankWall->SetPosition(tankWallPosition,fc);

    auto wallNode = tankWall->NewNode();
    wallNode->SetPositionInBody(Position(-0.45*tankLength,0.,0.75*tankDepth), fc);
    wallNode->RotateAroundYInBody(-90*DEG2RAD,fc);


    // --------------------------------------------------
    // Carriage definition
    // --------------------------------------------------

    auto carriage = system->NewBody();
    carriage->SetName("Carriage");
    makeItBox(carriage, 0.1*tankWidth, 1.1*tankWidth, 0.1*tankWidth, mass);

    auto carriageToWallNode = carriage->NewNode();
    carriageToWallNode->SetPositionInBody(Position(0.,-0.5*tankWidth,0.), fc);
    carriageToWallNode->RotateAroundYInBody(-90*DEG2RAD,fc);

    auto carriageToShipNode = carriage->NewNode();
    carriageToShipNode->SetPositionInBody(Position(0.,0.05*tankWidth,0.), fc);
    carriageToShipNode->RotateAroundYInBody(90*DEG2RAD,fc);
    carriageToShipNode->RotateAroundXInBody(90*DEG2RAD,fc);


    // --------------------------------------------------
    // Link definitions
    // --------------------------------------------------

    auto linkToShip = make_prismatic_revolute_link(carriageToShipNode, shipNode, system);

    auto rail = make_prismatic_link(wallNode, carriageToWallNode, system);

    return rail->Motorize(VELOCITY);

}


int main() {

    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system;

    // --------------------------------------------------
    // Ship
    // --------------------------------------------------

    auto ship = system.NewBody();
    ship->SetName("Ship");
    ship->AddMeshAsset("DTMB5512.obj");
    ship->SetColor(Green);

    // Inertia
    double mass = 86.0; double Ixx = 1.98; double Iyy = 53.88; double Izz = 49.99;
    Position COGPosition(0., 0., 0.03); // 0.03
    FrFrame COGFrame(COGPosition, FrRotation(), NWU);

    ship->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGFrame, NWU));

    auto shipNode = ship->NewNode();
    shipNode->SetPositionInBody(ship->GetCOG(fc),fc);
    shipNode->RotateAroundYInBody(90*DEG2RAD,fc);
    shipNode->RotateAroundXInBody(90*DEG2RAD,fc);

    // --------------------------------------------------
    // Carriage
    // --------------------------------------------------

    auto carriage = make_carriage(&system, shipNode);

    FrCosRampFunction ramp; ramp.SetByTwoPoints(0.,0.,10.,5.);

    carriage->SetMotorFunction(ramp);

    // Run

    system.Initialize();
    system.DoAssembly();

    system.SetTimeStep(0.01);

    system.RunInViewer(0., 10, false);
}