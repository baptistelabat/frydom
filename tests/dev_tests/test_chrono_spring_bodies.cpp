//
// Created by frongere on 19/02/2020.
//

#include "frydom/frydom.h"
#include "chrono_irrlicht/ChIrrApp.h"


using namespace chrono;

using namespace chrono::irrlicht;
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main() {

//  double diameter = 0.0332;
//  double A = 0.25 * MU_PI * diameter * diameter;
//  double E = 77.5e9;
//  double rho_line = 3121;
//  double linear_density = A * rho_line;
//
//  double L = 2;
//  double K = E*A/L;
//  double Cint = 100e6;
  double diameter = 0.0332;
  double A = 0.25 * MU_PI * diameter * diameter;
  double E = 1e3;
  double rho_line = 3121;
  double linear_density = A * rho_line;

  double L = 2;
  double K = E*A/L;
  double Cint = 1e5;


  ChSystemSMC system;

  auto world_body = std::make_shared<chrono::ChBody>();
  system.AddBody(world_body);
  world_body->SetBodyFixed(true);
//  auto anchor = std::make_shared<ChMarker>();
//  world_body->AddMarker(anchor);
//  anchor->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(0, 0, 0)));


  auto body1 = std::make_shared<chrono::ChBody>();
  system.AddBody(body1);
  body1->SetMass(10);
  ChVector<> position(0, 0, 0);
  body1->SetPos(position);
//  auto body1_anchor = std::make_shared<ChMarker>();
//  body1->AddMarker(body1_anchor);
//  body1_anchor->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(2, 0, 0)));
  auto sphere = std::make_shared<ChSphereShape>();
  sphere->GetSphereGeometry().center = position;
  sphere->GetSphereGeometry().rad = 10;
  body1->AddAsset(sphere);


  auto spring1 = std::make_shared<ChLinkSpring>();
  spring1->Set_SpringRestLength(2);
  spring1->Set_SpringK(K);
  spring1->Set_SpringR(Cint);
  spring1->Initialize(world_body, body1, true, ChVector<>(), ChVector<>(), false, 2.);
  system.AddLink(spring1);



  ChIrrApp application(&system);

//  application.AddTypicalSky();
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(0, 40, -60));

  application.AssetBindAll();
  application.AssetUpdateAll();
  application.SetTimestep(1e-3);

  while (application.GetDevice()->run()) {
    application.BeginScene(true, true, SColor(255,140,161,192));
    application.DrawAll();
    application.DoStep();
    application.EndScene();

    auto body_pos = body1->GetPos();
//    std::cout << body_pos.z() << std::endl;

  }











}
