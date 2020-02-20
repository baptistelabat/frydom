//
// Created by frongere on 19/02/2020.
//

#include <iostream>

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

  double diameter = 0.0332;
  double A = 0.25 * MU_PI * diameter * diameter;
  double E = 77.5e9;
  double rho_line = 3121;
  double linear_density = A * rho_line;

  double L = 2;
  double K = E * A / L;
  double Cint = 1e4;


  std::cout << "Stiffness: " << K << " N/m" << std::endl;

  ChSystemSMC system;

  auto world_body = std::make_shared<chrono::ChBody>();
  system.AddBody(world_body);
  world_body->SetBodyFixed(true);
//  auto anchor = std::make_shared<ChMarker>();
//  world_body->AddMarker(anchor);
//  anchor->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(0, 0, 0)));


  auto body1 = std::make_shared<chrono::ChBody>();
  system.AddBody(body1);
  body1->SetMass(140);
  ChVector<> position(0, -5.5, 0);
  body1->SetPos(position);
//  auto body1_anchor = std::make_shared<ChMarker>();
//  body1->AddMarker(body1_anchor);
//  body1_anchor->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(2, 0, 0)));
  auto sphere = std::make_shared<ChSphereShape>();
  sphere->GetSphereGeometry().center = ChVector<>();
  sphere->GetSphereGeometry().rad = 1;
  body1->AddAsset(sphere);


  auto spring1 = std::make_shared<ChLinkSpring>();
  spring1->Set_SpringRestLength(2);
  spring1->Set_SpringK(K);
  spring1->Set_SpringR(Cint);
  spring1->Initialize(world_body,
                      body1,
                      true,
                      ChVector<>(),
                      ChVector<>(),
                      false,
                      5.);
  system.AddLink(spring1);


  ChIrrApp application(&system);

//  application.AddTypicalSky();
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(0, 8, -12));

  application.AssetBindAll();
  application.AssetUpdateAll();
  application.SetTimestep(1e-2);


//  system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
//  system.SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);
//  system.SetTimestepperType(ChTimestepper::Type::NEWMARK);

//  system.SetTimestepperType(ChTimestepper::Type::HEUN);

  system.SetTimestepperType(ChTimestepper::Type::HHT);
  auto integrator = std::dynamic_pointer_cast<chrono::ChTimestepperHHT>(system.GetTimestepper());
  application.SetTimestep(5e-3); // Pour initialiser le pas de temps
  integrator->SetAlpha(-0.28); // Min is -0.33333333 == max damping
  integrator->SetMaxiters(8);
  integrator->SetAbsTolerances(5e-5, 1.8);
  integrator->SetMode(ChTimestepperHHT::POSITION);
  integrator->SetModifiedNewton(false);
  integrator->SetScaling(true);
//  integrator->SetStepControl(false);
//  integrator->SetVerbose(true);


  auto body_pos = body1->GetPos();
  std::cout << "Time: " << system.GetChTime() << "\tbody pos: " << body_pos.y() << std::endl;
  while (application.GetDevice()->run()) {
    application.BeginScene(true, true, SColor(255, 140, 161, 192));
    application.DrawAll();
    application.DoStep();
    application.EndScene();

    body_pos = body1->GetPos();
    std::cout << "Time: " << system.GetChTime() << "\tbody pos: " << body_pos.y() << std::endl;

  }


}
