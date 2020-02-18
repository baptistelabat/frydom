//
// Created by frongere on 30/01/2020.
//

#include "frydom/frydom.h"


using namespace frydom;

int main() {

  FrOffshoreSystem system("test_lumped_mass_cable");

  system.GetEnvironment()->GetOcean()->SetInfiniteDepth();

  auto world_body = system.GetWorldBody();

  auto anchor = world_body->NewNode("anchor");


  auto sphere = system.NewBody("sphere");
  makeItSphere(sphere, 10, 1000);
  sphere->SetPosition({50, 0., 0.}, NWU);
//  sphere->SetFixedInWorld(true);

  auto cylinder_anchor = sphere->NewNode("cylinder_anchor");

  double diameter = 0.0332;
  double linear_density = 0.25 * MU_PI * diameter * diameter * 3121;
  double E = 77.5e9;

  auto cable_properties = make_cable_properties(diameter, linear_density, E, 100e6);

  auto cat_cable = make_catenary_line("catenary_cable", anchor, cylinder_anchor, cable_properties, true, 100, WATER);

  auto cable = FrLumpedMassCable("cable",
                                 anchor,
                                 cylinder_anchor,
                                 cable_properties,
                                 100,
                                 30);

  system.Initialize();

  system.SetTimeStep(1e-3);
  system.RunInViewer();

  return 0;
}
