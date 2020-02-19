//
// Created by frongere on 19/02/2020.
//


#include "frydom/frydom.h"

using namespace frydom;

int main() {

  double diameter = 0.0332;
  double A = 0.25 * MU_PI * diameter * diameter;
  double E = 77.5e9;
  double rho_line = 3121;
  double linear_density = A * rho_line;

  double L = 2;
  double K = E * A / L;
  double Cint = 100e6;


  FrOffshoreSystem system("test");

  system.GetEnvironment()->GetOcean()->ShowSeabed(false);
  system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);

  auto anchor = system.GetWorldBody()->NewNode("anchor");
  system.GetWorldBody()->AddSphereShape(0.5, {0, 0, 0}, NWU);
  system.GetWorldBody()->AddSphereShape(0.5, {5, 0, 0}, NWU);

  auto node1 = system.GetWorldBody()->NewNode("1");
  node1->SetPositionInWorld({0., 0., 0.}, NWU);

  auto node2 = system.GetWorldBody()->NewNode("2");
  node2->SetPositionInWorld({5., 0., 0.}, NWU);

  auto props = make_cable_properties(diameter, linear_density, E, Cint);

  auto cable = std::make_shared<FrLumpedMassCable>("cable",
                                                   node1,
                                                   node2,
                                                   props,
                                                   4.8,
                                                   2);


  system.SetTimeStep(1e-3);
  system.RunInViewer(0., 20.);


  return 0;
}
