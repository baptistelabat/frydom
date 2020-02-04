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
#include <matplotlibcpp.h>

using namespace frydom;
using namespace chrono;

int main(int argc, char *argv[]) {

  // -------------------------------------------------------
  // System
  // -------------------------------------------------------

  FrOffshoreSystem system;

  // -------------------------------------------------------
  // Environment
  // -------------------------------------------------------

  system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_REGULAR);
  auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
  waveField->SetRegularWaveHeight(3.);
  waveField->SetRegularWavePeriod(9.);
  waveField->SetMeanWaveDirection(0, DEG);

  waveField->GetSteadyElevation(0, 0);

  // -------------------------------------------------------
  // Wave probe
  // -------------------------------------------------------

  double vx = 1.;

  //auto waveProbe = waveField->NewWaveProbe(0., 0.);
  //waveProbe->Initialize();

  auto node = std::make_shared<FrNode>();
  node->GetPos().x() = 0.;
  node->GetPos().y() = 0.;
  node->GetPos().z() = 0.;
  node->GetPos_dt().x() = vx;

  auto waveProbe = std::make_shared<FrLinearWaveProbe>();
  waveProbe->AttachedNode(node);
  waveProbe->SetWaveField(waveField);
  waveField->AddWaveProbe(waveProbe);
  waveProbe->Initialize();

  auto node_fixed = std::make_shared<FrNode>();
  node_fixed->GetPos().x() = 0.;
  node_fixed->GetPos().y() = 0.;
  node_fixed->GetPos().z() = 0.;

  auto waveProbe_fixed = std::make_shared<FrLinearWaveProbe>();
  waveProbe_fixed->AttachedNode(node_fixed);
  waveProbe_fixed->SetWaveField(waveField);
  waveField->AddWaveProbe(waveProbe_fixed);
  waveProbe_fixed->Initialize();

  // -------------------------------------------------------
  // Simulation
  // -------------------------------------------------------

  std::vector<double> vtime, eta, x, eta_0, x_0;

  double time = 0.;
  double dt = 0.1;

  system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
  system.SetStep(dt);
  system.Initialize();

  while (time < 150) {

    system.DoStepDynamics(dt);
    node->Move(ChVector<double>(vx * dt, 0., 0.));
    time += dt;

    vtime.push_back(time);
    eta.push_back(waveProbe->GetElevation(time));
    x.push_back(waveProbe->GetX());
    eta_0.push_back(waveProbe_fixed->GetElevation(time));
    x_0.push_back(waveProbe_fixed->GetX());

  }

  matplotlibcpp::subplot(2, 1, 1);
  matplotlibcpp::named_plot("eta_0", vtime, eta_0, "--");
  matplotlibcpp::named_plot("eta", vtime, eta);
  matplotlibcpp::legend();
  matplotlibcpp::xlabel("time (s)");
  matplotlibcpp::ylabel("eta (m)");

  matplotlibcpp::subplot(2, 1, 2);
  matplotlibcpp::named_plot("x_0", vtime, x_0, "--");
  matplotlibcpp::named_plot("x", vtime, x);
  matplotlibcpp::legend();
  matplotlibcpp::xlabel("time (s)");
  matplotlibcpp::ylabel("position (m)");

  matplotlibcpp::show();


}