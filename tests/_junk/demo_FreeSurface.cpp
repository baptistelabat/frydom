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

bool viz = true;
float friction = 0.6f;
double step_size = 1e-2;
bool capture_video = false;

using namespace frydom;

int main(int argc, char *argv[]) {

  // =================================================================================================================
  // OFFSHORE SYSTEM
  // =================================================================================================================
  FrOffshoreSystem system;

  // =================================================================================================================
  // FREE SURFACE
  // =================================================================================================================
  system.GetEnvironment()->GetFreeSurface()->SetGrid(-400, 400, 100, -50, 50, 100);

  // =================================================================================================================
  // CURRENT
  // =================================================================================================================
  system.GetEnvironment()->GetCurrent()->Set(EAST, 5, NED, GOTO, KNOT);

  // =================================================================================================================
  // SHIP
  // =================================================================================================================
  // Creating a body that has to be a floating body
  auto ship = std::make_shared<frydom::FrShip>();

  ship->SetName("my_ship");
  ship->SetIdentifier(1);
  double mass = 5e6;
  ship->SetMass(mass);
  ship->SetPos(chrono::ChVector<>(-200, 0, 0));

  auto rot = chrono::Q_from_AngAxis(0. * M_PI / 180., chrono::ChVector<>(0, 0, 1));

  ship->SetRot(rot);

  auto ship_velocity = ship->TransformDirectionLocalToParent(chrono::ChVector<>(5, 0, 0));
  ship->SetPos_dt(ship_velocity);
//    ship->SetRot_dt(chrono::ChQuaternion<>(0.3, chrono::ChVector<>(0, 1, 0)));


  ship->SetBodyFixed(false); // TODO: debloquer
//    auto material = std::make_shared<chrono::ChMaterialSurfaceNSC>();
//    ship->SetMaterialSurface(material);auto

  ship->SetInertiaXX(chrono::ChVector<>(1e5, 5e6, 5e6));

  // Defining the hydro mesh and as an asset
  ship->SetHydroMesh("MagneViking.obj", true);


  ship->SetCollide(false); // TODO: essayer avec..
  // Adding the ship to the system
  system.AddBody(ship);

  // =================================================================================================================
  // FORCES
  // =================================================================================================================

  // Creating a "propulsion force"
//    auto force2 = std::make_shared<frydom::FrTryalForce>();
//    ship->AddForce(force2); // Toujours ajouter la force au corps avant de la tuner !!!
//    auto force_asset = std::make_shared<frydom::FrForceAsset>(force2);
//    ship->AddAsset(force_asset);



  // Creating an ITTC57 force
//    auto force_ittc = std::make_shared<frydom::FrITTC57>();
//    ship->AddForce(force_ittc);
//    force_ittc->SetCharacteristicLength(80.);
//    force_ittc->SetHullFormFactor(0.1);
//    force_ittc->SetHullWettedSurface(2700.);


  // Creating a current force
//    auto current_force = std::make_shared<frydom::FrCurrentForce>();
//    ship->AddForce(current_force);


  ///===========================================================================================================
  /// 3 DOF CONSTRAINT
  ///===========================================================================================================

  ship->Set3DOF_ON();

  ///===========================================================================================================
  /// VISUALIZATION WITH IRRLICHT
  ///===========================================================================================================


  system.Initialize();

  // Visualization with irrlicht
  auto app = FrIrrApp(system, 200);
  app.Run();

  std::cout << "LEAVING MAIN PROGRAM" << "\n";
  return 0;

}
