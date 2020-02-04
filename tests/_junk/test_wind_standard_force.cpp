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
#include "matplotlibcpp.h"

using namespace frydom;
using namespace chrono;

int main(int argc, char *argv[]) {

  // System
  FrOffshoreSystem system;

  // Environment
  system.GetEnvironment()->SetWind(FrWind::UNIFORM);

  // Body
  auto mybody = std::make_shared<FrHydroBody>();
  system.Add(mybody);

  // Drag force
  auto drag_force = std::make_shared<FrWindStandardForce>();
  drag_force->SetLateralArea(10.);
  drag_force->SetTransverseArea(4.);
  drag_force->SetLpp(10.);
  drag_force->SetXc(1.);

  mybody->AddForce(drag_force);

  // Initialize
  system.Initialize();

  // Output

  auto theta = -180.;
  auto dtheta = 10.;

  std::vector<double> vfx, vfy, vmz;
  std::vector<double> vtheta;

  while (theta < 185.) {

    system.GetEnvironment()->GetWind()->Set(theta, 5.14, DEG, MS, NWU, COMEFROM);
    system.Initialize();
    drag_force->UpdateState();

    vtheta.push_back(theta);
    vfx.push_back(drag_force->GetForce().x());
    vfy.push_back(drag_force->GetForce().y());
    vmz.push_back(drag_force->GetTorque().z());

    theta += dtheta;

  }

  // Output

  matplotlibcpp::subplot(2, 2, 1);
  matplotlibcpp::plot(vtheta, vfx);
  matplotlibcpp::xlabel("theta (deg)");
  matplotlibcpp::ylabel("Fx (N)");
  matplotlibcpp::grid(true);

  matplotlibcpp::subplot(2, 2, 2);
  matplotlibcpp::plot(vtheta, vfy);
  matplotlibcpp::xlabel("theta (deg)");
  matplotlibcpp::ylabel("Fy (N)");
  matplotlibcpp::grid(true);

  matplotlibcpp::subplot(2, 2, 3);
  matplotlibcpp::plot(vtheta, vmz);
  matplotlibcpp::xlabel("theta (deg)");
  matplotlibcpp::ylabel("Mz (N.m)");
  matplotlibcpp::grid(true);

  matplotlibcpp::show();

}