//
// Created by frongere on 30/01/2020.
//

#include "frydom/frydom.h"


using namespace frydom;

int main() {

  FrOffshoreSystem system("test_lumped_mass_cable");

  system.GetEnvironment()->GetOcean()->ShowSeabed(true);
  // FIXME: le no show seabed ne doit pas declencher de profondeur infine !!! Ca doit seulement concerner l'asset !!
  system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(-100, NWU);
  system.GetEnvironment()->GetOcean()->GetSeabed()->GetSeabedGridAsset()->SetGrid(-500, 500, 500, -50, 50, 50);

  system.GetEnvironment()->GetOcean()->ShowFreeSurface(true);
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-500, 500, 500, -50, 50,
                                                                                            50);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(90, 2, DEG, KNOT, NED, GOTO);

  auto world_body = system.GetWorldBody();

  auto anchor = world_body->NewNode("anchor");


  auto sphere = system.NewBody("sphere");
  makeItSphere(sphere, 1, 1000);
  sphere->SetPosition({500, 0., -100.}, NWU);
  sphere->SetFixedInWorld(true);


  auto cylinder_anchor = sphere->NewNode("cylinder_anchor");

//  double cable_length = 64 + 425;
  double cable_length = 64 + 425 +30; // Adding more length...
//  double cable_length = 515;
  int nb_elements = 20; // TODO: voir dans quelle mesure on peut augmenter la discretisation


  auto cable_properties = make_cable_properties();

//  // DONNEES A-FLOWT
  cable_properties->SetLinearDensity(
      141); // FIXME: submerged weight = 122... Comment le prendre en compte comme cela ??? --> trouver un diametre equivalent...
  cable_properties->SetDiameter(0.168); // Vrai valeur
  //  cable_properties->SetEA(602.59e6); // Vrai valeur // FIXME: EA est systematiquement calcule alors que c'est ca qu'on veut...
  cable_properties->SetYoungModulus(602.58e6 / cable_properties->GetSectionArea());
  cable_properties->SetDragCoefficients(1.2, 0.);
  cable_properties->SetAddedMassCoefficients(2, 0.);
  cable_properties->SetHydrodynamicDiameter(0.168);
  cable_properties->SetRayleighDamping(1e4);


  // TODO: implementer le clump weight

  std::cout << "Cable total theoretical mass: " << cable_length * cable_properties->GetLinearDensity() << std::endl;

//  auto cable_properties = make_cable_properties(diameter, linear_density, E, rayleighDamping);

  auto cat_cable = make_catenary_line("catenary_cable", anchor, cylinder_anchor,
                                      cable_properties, true, cable_length); // TODO: devrait y avoir une detection auto du fluide...

  auto cable = FrLumpedMassCable("cable",
                                 anchor,
                                 cylinder_anchor,
                                 cable_properties,
                                 cable_length,
                                 nb_elements);

  cable.UpdateNodesMasses();

  cable.SetSpeedLimit(10);
  cable.ActivateSpeedLimit(true);


  std::cout << "Cable total numerical mass: " << cable.GetMass()
            << std::endl; // FIXME: il manque la moitier de la masse de chacun des elements aux frontieres...


  std::cout << "Unstretched length: " << cable.GetUnstretchedLengthFromElements() << std::endl;


  system.SetTimeStepper(FrOffshoreSystem::TIME_STEPPER::RUNGEKUTTA45);


  system.Initialize();

  system.SetTimeStep(1e-2);
  system.RunInViewer();

  return 0;
}
