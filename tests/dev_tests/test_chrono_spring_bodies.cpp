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


double diameter = 0.0332;
double A = 0.25 * MU_PI * diameter * diameter;
double E = 77.5e9;
double rho_line = 3121;
double linear_density = A * rho_line;

//double L = 3.33;
//double K = E * A / L;
double Cint = 1e5;
double damping = Cint * A;
//double damping = Cint / A;



std::shared_ptr<ChBody> NewElement(std::shared_ptr<ChBody> prev_body, double L) {

  auto system = prev_body->GetSystem();

  auto new_body = std::make_shared<chrono::ChBody>();
  system->AddBody(new_body);

  double lumped_mass = linear_density * L;

//  lumped_mass = 50;
//  std::cout << lumped_mass << std::endl;

  new_body->SetMass(lumped_mass);
  ChVector<> position(0.0, -L*1.000, 0);
  new_body->SetPos(prev_body->GetPos() + position);

  new_body->SetMaxSpeed(10); // info: Permet de stabiliser de maniere artificielle !!!
  new_body->SetLimitSpeed(true);
  // TODO: ca stabilise bien et permet de monter en dt. L'idee serait alors a chaque pas de temps de prendre
  // la vitesse du fairlead et de regler le maxspeed des noeuds LM a x% de sa vitesse...


//  auto body1_anchor = std::make_shared<ChMarker>();
//  body1->AddMarker(body1_anchor);
//  body1_anchor->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(2, 0, 0)));
  auto sphere = std::make_shared<ChSphereShape>();
  sphere->GetSphereGeometry().center = ChVector<>(); // Attention, la position est relative au corps ici...
  sphere->GetSphereGeometry().rad = 0.5;
  new_body->AddAsset(sphere);


  auto spring1 = std::make_shared<ChLinkSpring>();
  spring1->Set_SpringRestLength(L);
  spring1->Set_SpringK( E*A / L);
  spring1->Set_SpringR(damping);
  spring1->Initialize(prev_body, // TODO: utiliser les marker a la place comme fait dans LM...
                      new_body,
                      true,
                      ChVector<>(),
                      ChVector<>(),
                      false,
                      L);
  system->AddLink(spring1);

  return new_body;

}



int main() {

  ChSystemSMC system;

  auto world_body = std::make_shared<chrono::ChBody>();
  system.AddBody(world_body);
  world_body->SetBodyFixed(true);

  double l = 5;

  auto body1 = NewElement(world_body, l);
  auto body2 = NewElement(body1, l);
  auto body3 = NewElement(body2, l);
  auto body4 = NewElement(body3, l);
  auto body5 = NewElement(body4, l);
  auto body6 = NewElement(body5, l);
  auto body7 = NewElement(body6, l);
  auto body8 = NewElement(body7, l);
  auto body9 = NewElement(body8, l);
  auto body10 = NewElement(body9, l);
  auto body11 = NewElement(body10, l);
  auto body12 = NewElement(body11, l);
  auto body13 = NewElement(body12, l);
  auto body14 = NewElement(body13, l);
  auto body15 = NewElement(body14, l);
  auto body16 = NewElement(body15, l);
  auto body17 = NewElement(body16, l);
  auto body18 = NewElement(body17, l);
  auto body19 = NewElement(body18, l);
  auto body20 = NewElement(body19, l);
  auto body21 = NewElement(body20, l);


  #define IRRLICHT


#ifdef IRRLICHT
  //  Irrlicht
  ChIrrApp application(&system);

  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(0, 50, -70));

  application.AssetBindAll();
  application.AssetUpdateAll();

#endif

  double timestep = 1e-3;


//  application.SetTimestep(timestep);






// Clairement, la liaison spring n'introduit aucune contrainte. Le solveur n'est donc pas utilisé !!!
//  system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
//  system.SetSolverType(ChSolver::Type::MINRES);
//  system.GetSolver()->SetVerbose(true);


//  system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
//  system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
//  system.SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);


//  system.SetTimestepperType(ChTimestepper::Type::HEUN);


//  system.SetTimestepperType(ChTimestepper::Type::NEWMARK);
//  auto integrator = std::dynamic_pointer_cast<chrono::ChTimestepperNewmark>(system.GetTimestepper());
//  integrator->SetGammaBeta(0.75, 0.25);




  system.SetTimestepperType(ChTimestepper::Type::HHT);
  auto integrator = std::dynamic_pointer_cast<chrono::ChTimestepperHHT>(system.GetTimestepper());
  #ifdef IRRLICHT
  application.SetTimestep(timestep); // Pour initialiser le pas de temps
  #endif
  integrator->SetAlpha(-0.25); // Min is -0.33333333 == max damping
//  integrator->SetMaxiters(8);
  integrator->SetMaxiters(30);
  integrator->SetAbsTolerances(5e-5, 1.8);
//  integrator->SetAbsTolerances(1e-2, 1.8);
//  integrator->SetMode(ChTimestepperHHT::POSITION);
  integrator->SetMode(ChTimestepperHHT::POSITION);
//  integrator->SetModifiedNewton(false);
  integrator->SetModifiedNewton(true);

  integrator->SetScaling(true);
  integrator->SetStepControl(true);
//  integrator->SetVerbose(true);


  auto body_pos = body1->GetPos();


#ifdef IRRLICHT
  std::cout << "Time: " << system.GetChTime() << "\tbody pos: " << body_pos.y() << std::endl;
  while (application.GetDevice()->run()) {
    application.BeginScene(true, true, SColor(255, 140, 161, 192));
    application.DrawAll();
    application.DoStep();
    application.EndScene();

    body_pos = body1->GetPos();
    std::cout << "Time: " << system.GetChTime() << "\tbody pos: " << body_pos.y() << "\tVel: " << body1->GetPos_dt().Length() << std::endl;
//    std::cout << spring1->Get_SpringReact() << std::endl;
//    std::cout << system.GetTimerSolver() << std::endl;
//    std::cout << system.GetSolver()

  }
#else

  while (true) {
    system.DoStepDynamics(timestep);

    body_pos = body1->GetPos();
    std::cout << "Time: " << system.GetChTime() << "\tbody pos: " << body_pos.y() << "\tVel: " << body1->GetPos_dt().Length() << std::endl;

  }

#endif


}
