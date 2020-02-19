//
// Created by frongere on 19/02/2020.
//


#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test");

  system.GetEnvironment()->GetOcean()->ShowSeabed(false);
  system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);

  auto anchor = system.GetWorldBody()->NewNode("anchor");
  system.GetWorldBody()->AddSphereShape(0.5, {0, 0, 0}, NWU);
  system.GetWorldBody()->AddSphereShape(0.5, {5, 0, 0}, NWU);

//  auto sphere = make_SphereBody("sphere", &system, 1, 10);
//  sphere->SetPosition({1, 0, 0}, NWU);
//
//
//  auto cable = std::make_shared<FrLumpedMassCable>();


  system.RunInViewer(0., 10.);





  return 0;
}
