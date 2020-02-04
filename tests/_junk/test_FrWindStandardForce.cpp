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

int main(int argc, char *argv[]) {

  FrOffshoreSystem_ system;

  system.GetEnvironment()->GetAtmosphere()->GetWind()->MakeFieldUniform();

  auto body = system.NewBody();

  double As = 0.05;
  double Af = 0.0096;
  double Loa = 0.75;
  double Lpp = 0.71;
  double xc = 0.;
  double uref = 25.;

  auto force = std::make_shared<FrWindStandardForce_>();
  force->SetLenghtBetweenPerpendicular(Lpp);
  force->SetLateralArea(As);
  force->SetTransverseArea(Af);
  force->SetXCenter(xc);

  body->AddExternalForce(force);

  // Output

  double theta = -180.;
  double dtheta = 10.;

  double rho = system.GetEnvironment()->GetAtmosphere()->GetDensity();
  double xadim = 0.5 * rho * uref * uref * Af;
  double yadim = 0.5 * rho * uref * uref * As;
  double nadim = 0.5 * rho * uref * uref * As * Loa;

  std::vector<double> vfx, vfy, vmz;
  std::vector<double> vtheta;

  while (theta < 185.) {

    system.GetEnvironment()->GetAtmosphere()->GetWind()->GetFieldUniform()->Set(theta, uref, DEG, MS, NED, COMEFROM);

    force->Update(0.);
    vtheta.push_back(theta);
    vfx.push_back(force->GetForceInWorld(NWU).GetFx() / xadim);
    vfy.push_back(force->GetForceInWorld(NWU).GetFy() / yadim);
    vmz.push_back(force->GetTorqueInWorldAtCOG(NWU).GetMz() / nadim);

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
  matplotlibcpp::ylabel("Fy (N");
  matplotlibcpp::grid(true);

  matplotlibcpp::subplot(2, 2, 3);
  matplotlibcpp::plot(vtheta, vmz);
  matplotlibcpp::xlabel("theta (deg)");
  matplotlibcpp::ylabel("Mz (N.m)");
  matplotlibcpp::grid(true);

  matplotlibcpp::show();

  // CSV

  std::ofstream file;
  file.open("StandardWindForce.csv");
  file << "direction;cx;cy;cn" << std::endl;
  for (int i = 0; i < vtheta.size(); i++) {
    file << vtheta[i] << ";" << vfx[i] << ";" << vfy[i] << ";" << vmz[i] << std::endl;
  }
  file.close();

}