//
// Created by frongere on 30/01/2020.
//

#include "frydom/frydom.h"


using namespace frydom;

int main() {

  FrOffshoreSystem system("test_lumped_mass_cable");

  auto world_body = system.GetWorldBody();

  auto anchor = world_body->NewNode("anchor");


  auto cylinder = system.NewBody("cylinder");
  makeItCylinder(cylinder, 1, 1, 10);
  cylinder->SetPosition({100, 0., 0.}, NWU);

  auto cylinder_anchor = cylinder->NewNode("cylinder_anchor");


  auto cable_properties = make_cable_properties(0.02, 1., 1e9);



  auto cable = FrLumpedMassCable("cable",
                                 anchor,
                                 cylinder_anchor,
                                 cable_properties,
                                 100,
                                 10);


  return 0;
}
