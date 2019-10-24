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

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_logging");

  auto sphere = system.NewBody("sphere");
  makeItSphere(sphere, 5, 100);
  sphere->SetColor(Red);

  sphere->SetPosition({0, 0, 0}, NWU);
//  sphere->SetFixedInWorld(true);
  sphere->SetVelocityInWorldNoRotation({0, 5, 15}, NWU);

  system.SetTimeStep(0.03);
  system.MonitorRealTimePerfs(true);

//  system.RunInViewer();

  system.RunDynamics(0.03);



//  system.RunDynamics(0.04);
//  system.


  return 0;
}
