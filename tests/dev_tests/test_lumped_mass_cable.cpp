//
// Created by frongere on 30/01/2020.
//

#include "frydom/frydom.h"


using namespace frydom;

int main() {

  FrOffshoreSystem system("test_lumped_mass_cable");

  system.GetEnvironment()->GetOcean()->ShowSeabed(true);
  system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(-100, NWU);
  system.GetEnvironment()->GetOcean()->GetSeabed()->GetSeabedGridAsset()->SetGrid(-500, 500, 500, -500, 500, 500);

  system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);

  auto world_body = system.GetWorldBody();

  auto anchor = world_body->NewNode("anchor");


  auto sphere = system.NewBody("sphere");
  makeItSphere(sphere, 1, 1000);
  sphere->SetPosition({500, 0., -100.}, NWU);
  sphere->SetFixedInWorld(true);


  auto cylinder_anchor = sphere->NewNode("cylinder_anchor");

//  double diameter = 0.0332;
//  double section = 0.25 * MU_PI * diameter * diameter;
//  double linear_density = section * 3121;
////  double E = 77.5e9;
//  double E = 77.5e9;
//  double rayleighDamping = 1e6;
//  double cable_length = 64 + 425;
  double cable_length = 600;
  int nbElements = 10;



  auto cable_properties = make_cable_properties();

  cable_properties->SetLinearDensity(141); // FIXME: submerged weight = 122... Comment le prendre en compte comme cela ??? --> trouver un diametre equivalent...
//  cable_properties->SetEA(602.59e6); // Vrai valeur
  cable_properties->SetEA(602.59e3);
  cable_properties->SetDragCoefficients(1.2, 0.);
  cable_properties->SetAddedMassCoefficients(2, 0.);
  cable_properties->SetHydrodynamicDiameter(0.168);
  cable_properties->SetDiameter(0.168);
  cable_properties->SetRayleighDamping(1e4);


  // TODO: implementer le clump weight

  std::cout << "Cable total theoretical mass: " << cable_length * cable_properties->GetLinearDensity() << std::endl;

//  auto cable_properties = make_cable_properties(diameter, linear_density, E, rayleighDamping);

  auto cat_cable = make_catenary_line("catenary_cable", anchor, cylinder_anchor,
      cable_properties, true, cable_length, AIR);

  auto cable = FrLumpedMassCable("cable",
                                 anchor,
                                 cylinder_anchor,
                                 cable_properties,
                                 cable_length,
                                 nbElements);

  cable.UpdateNodesMasses();

  cable.SetSpeedLimit(1.);
  cable.ActivateSpeedLimit(true);


  std::cout << "Cable total numerical mass: " << cable.GetMass() << std::endl;


  std::cout << "Unstretched length: " << cable.GetUnstretchedLengthFromElements() << std::endl;


  system.SetTimeStepper(FrOffshoreSystem::TIME_STEPPER::RUNGEKUTTA45);


  system.Initialize();

  system.SetTimeStep(1e-4);
  system.RunInViewer();

  return 0;
}
