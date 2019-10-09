//
// Created by frongere on 07/10/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_logging");

  auto sphere = system.NewBody("sphere");
  makeItSphere(sphere, 1, 1000);
  sphere->SetColor(Red);

  sphere->SetPosition({0, 0, 0}, NWU);
//  sphere->SetFixedInWorld(true);
  sphere->SetVelocityInWorldNoRotation({0, 0, 15}, NWU);

  system.SetTimeStep(0.1);
  system.MonitorRealTimePerfs(true);

  system.RunInViewer();



//  system.RunDynamics(0.04);
//  system.




  return 0;
}
