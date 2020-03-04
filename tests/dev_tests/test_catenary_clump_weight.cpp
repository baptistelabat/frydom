//
// Created by frongere on 04/03/2020.
//


#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_catenary_clumpweight");

//  system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(-100, NWU);
//  system.GetEnvironment()->GetOcean()->GetSeabed()->GetSeabedGridAsset()->SetGrid(-50, 600, 50, -50, 50, 50);
//  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-50, 600, 50, -50, 50, 50);
  system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);
  system.GetEnvironment()->GetOcean()->ShowSeabed(false);

  auto world_body = system.GetWorldBody();

  auto node1 = world_body->NewNode("node1");
  node1->SetPositionInWorld({500, 0, 0}, NWU);

  auto node2 = world_body->NewNode("node2");
  node2->SetPositionInWorld({0, 0, -100}, NWU);


  auto cable_properties = make_cable_properties();

//  // DONNEES A-FLOWT
  cable_properties->SetLinearDensity(141);
  cable_properties->SetDiameter(0.168); // Vrai valeur
  //  cable_properties->SetEA(602.59e6); // Vrai valeur
  cable_properties->SetYoungModulus(602.58e6 / cable_properties->GetSectionArea());
  cable_properties->SetDragCoefficients(1.2, 0.);
  cable_properties->SetAddedMassCoefficients(2, 0.);
  cable_properties->SetHydrodynamicDiameter(0.168);
  cable_properties->SetRayleighDamping(1e4);

  auto cable = make_catenary_line("cable", node2, node1, cable_properties, true, 520, AIR);




  system.RunInViewer();


  return 0;
}
