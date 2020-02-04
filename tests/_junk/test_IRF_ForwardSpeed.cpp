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

#include <matplotlibcpp.h>
#include "frydom/frydom.h"

#include <iostream>
#include <fstream>

using namespace frydom;

void PlotResults(const std::vector<double> &vtime, const std::vector<double> &vx,
                 const std::vector<double> &vy, const std::vector<double> &vz,
                 const std::string label = "velocity", const std::string ylabel = "", const std::string line = "-",
                 const bool show = false) {

  matplotlibcpp::named_plot(label + ".x", vtime, vx, line);
  matplotlibcpp::named_plot(label + ".y", vtime, vy, line);
  matplotlibcpp::named_plot(label + ".z", vtime, vz, line);
  matplotlibcpp::xlabel("time (s)");
  matplotlibcpp::ylabel(ylabel + " (SI)");
  matplotlibcpp::grid;
  matplotlibcpp::legend();
  if (show) { matplotlibcpp::show(); }
}

void PlotResults(const std::vector<double> &vtime, const std::vector<chrono::ChVector<>> &vdata,
                 const std::string label = "position", const std::string ylabel = "",
                 const std::string line = "-", const bool show = false) {

  std::vector<double> vx, vy, vz;
  for (auto &data : vdata) {
    vx.push_back(data.x());
    vy.push_back(data.y());
    vz.push_back(data.z());
  }

  PlotResults(vtime, vx, vy, vz, label, ylabel, line, show);

}


std::shared_ptr<FrShip> Platform(FrOffshoreSystem *system) {

  auto platform = std::make_shared<FrShip>();
  platform->SetName("Deepsea_Stavanger");
  platform->SetHydroMesh("Platform_GVA7500.obj", true);
  platform->SetLpp(108.8);
  platform->SetMass(3.22114e7);
  platform->SetCOG(chrono::ChVector<double>(0., 0., 8.65));
  platform->SetInertiaXX(chrono::ChVector<double>(5e7, 5.58e10, 1e8)); // TODO : verifier les valeurs inertielles
  platform->SetPos(
      chrono::ChVector<double>(0., 0., 8.65)); // FIXME : il s'agit ici de la position du centre de gravité

  system->AddBody(platform);

  // Hydrostatic load

  auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
  auto hstStiffness = hstForce->GetStiffnessMatrix();
  double K33 = 1e7;
  double K44 = 1e9;
  hstStiffness->SetDiagonal(K33, K44, K44);
  platform->AddForce(hstForce);

  // Hydrodynamic load
  system->SetHydroDB("Platform_HDB.hdb5");
  auto HydroMapIndex = system->GetHydroMapNb() - 1;
  system->GetHydroMapper(HydroMapIndex)->Map(platform, 0);

  auto radModel = std::make_shared<FrRadiationConvolutionModel>(system->GetHydroDB(HydroMapIndex), system);
  radModel->SetHydroMapIndex(HydroMapIndex); // TODO : patch hydro map multibody
  radModel->AddRadiationForceToHydroBody(platform);
  radModel->SetSpeedDependent(false);

  // Wind load
  auto wind_force = std::make_shared<FrWindForce>("Platform_PolarWindCoeffs_NC.yml");
  platform->AddForce(wind_force);

  // Current load
  auto current_force = std::make_shared<FrCurrentForce>("Platform_PolarCurrentCoeffs_NC.yml");
  platform->AddForce(current_force);

  // Additional damping

  //auto lin_damping_force = std::make_shared<FrLinearDamping>();
  //lin_damping_force->SetManeuveuringDampings(1e7, 1e7, 1e8);
  //platform->AddForce(lin_damping_force);

  //auto lin_damping_force = std::make_shared<FrLinearDamping>();
  //lin_damping_force->SetSeakeepingDampings(1e7, 1e7, 1e8);
  //platform->AddForce(lin_damping_force);

  // Wave Probe
  auto waveField = system->GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
  auto waveProbe = waveField->NewWaveProbe(0., 0.);
  waveProbe->Initialize();

  // Wave Drift Force
  auto DriftForce = std::make_shared<FrWaveDriftForce>("Platform_WaveDriftCoeff.h5");
  platform->AddForce(DriftForce);
  DriftForce->SetBody(platform);
  DriftForce->SetWaveProbe(waveProbe);

  // Wave Excitation force
  auto excForce = std::make_shared<FrLinearExcitationForce>();
  platform->AddForce(excForce);
  excForce->SetWaveProbe(waveProbe);
  excForce->SetHydroMapIndex(HydroMapIndex);


  return platform;
};

std::shared_ptr<FrShip> Ship(FrOffshoreSystem *system) {

  auto ship_pos = chrono::ChVector<double>(0., 0., 0.);

  auto ship = std::make_shared<FrShip>();
  ship->SetName("ship");
  ship->SetHydroMesh("ship.obj", true);
  ship->SetLpp(76.20);
  ship->SetMass(8.531e6);
  ship->SetCOG(chrono::ChVector<double>(0., 0., 0.));
  ship->SetInertiaXX(chrono::ChVector<double>(1.02e8, 2.8e9, 2.8e9));
  ship->SetPos(ship_pos);

  system->AddBody(ship);

  // Hydrostatic load
  auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
  auto hstStiffness = hstForce->GetStiffnessMatrix();
  double K33 = 1.61e7;
  double K44 = 2.44e8;
  double K55 = 7.42e9;
  hstStiffness->SetDiagonal(K33, K44, K55);
  ship->AddForce(hstForce);

  // Hydrodynamic load
  system->SetHydroDB("ship.h5");
  auto HydroMapIndex = system->GetHydroMapNb() - 1;
  system->GetHydroMapper(HydroMapIndex)->Map(ship, 0);

  auto radModel = std::make_shared<FrRadiationConvolutionModel>(system->GetHydroDB(HydroMapIndex), system);
  radModel->SetHydroMapIndex(HydroMapIndex); // TODO : patch hydro map multibody
  radModel->AddRadiationForceToHydroBody(ship);

  // Wind load
  //auto wind_force = std::make_shared<FrWindForce>("Ship_PolarWindCoeffs.yml");
  //ship->AddForce(wind_force);

  // Current load
  //auto current_force = std::make_shared<FrCurrentForce>("Ship_PolarCurrentCoeffs.yml");
  //ship->AddForce(current_force);

  // Wave Probe
  auto waveField = system->GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
  auto waveProbe = waveField->NewWaveProbe(ship_pos.x(), ship_pos.y());
  waveProbe->Initialize();

  // Wave Drift Force
  //auto DriftForce = std::make_shared<FrWaveDriftForce>("Ship_WaveDriftCoeff.h5");
  //ship->AddForce(DriftForce);
  //DriftForce->SetBody(ship);
  //DriftForce->SetWaveProbe(waveProbe);
  //DriftForce->SetCmplxElevation();

  // Wave Excitation force
  auto excForce = std::make_shared<FrLinearExcitationForce>();
  ship->AddForce(excForce);
  excForce->SetWaveProbe(waveProbe);
  excForce->SetHydroMapIndex(HydroMapIndex);

  // Additional damping

  auto lin_damping_force = std::make_shared<FrLinearDamping>();
  lin_damping_force->SetDiagonalDamping(1e7, 1e7, 0, 0, 0, 1e8);
  ship->AddForce(lin_damping_force);

  //auto lin_damping_force = std::make_shared<FrLinearDamping>();
  //lin_damping_force->SetSeakeepingDampings(1e7, 1e7, 1e8);
  //ship->AddForce(lin_damping_force);

  //auto test_force = std::make_shared<FrTestForce>();
  //ship->AddForce(test_force);

  return ship;
}

int main(int argc, char *argv[]) {

  // -----------------------------------------------
  // System
  // -----------------------------------------------

  FrOffshoreSystem system;

  // --------------------------------------------------------
  // Environment
  // --------------------------------------------------------
  // Uniform wind
  system.GetEnvironment()->SetWind(FrWind::UNIFORM);
  system.GetEnvironment()->GetWind()->Set(NORTH, 0., KNOT, NED, COMEFROM);

  // Uniform current
  system.GetEnvironment()->SetCurrent(FrCurrent::UNIFORM);
  system.GetEnvironment()->GetCurrent()->Set(EAST, 0., KNOT, NED, COMEFROM);

  // Irregular wave
  system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_IRREGULAR);
  auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
  waveField->SetMeanWaveDirection(0., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
  waveField->SetWavePulsations(0.5, 2., 80, RADS);
  waveField->GetWaveSpectrum()->SetHs(2.);
  waveField->GetWaveSpectrum()->SetTp(10.);
  waveField->GetWaveRamp()->Deactivate();
  waveField->GetSteadyElevation(0, 0);

  // -----------------------------------------------------
  // Body
  // -----------------------------------------------------
  auto ship = Platform(&system);
  ship->SetPos_dt(chrono::ChVector<double>(0.5, 0., 0.));
  ship->SetEquilibriumFrame(MeanMotion, 60.);

  // ------------------------------------------------------
  // Simulation
  // ------------------------------------------------------

  std::vector<chrono::ChVector<double>> position_body, velocity_body;
  std::vector<chrono::ChVector<double>> steady_velocity;
  std::vector<chrono::ChVector<double>> pert_velocity;
  std::vector<double> vtime;

  double time = 0.;
  double dt = 0.1;

  system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
  system.SetStep(dt);
  system.Initialize();

  while (time < 500.) {

    system.DoStepDynamics(dt);
    time += dt;

    vtime.push_back(time);

    position_body.push_back(ship->GetPosition());
    velocity_body.push_back(ship->GetVelocity());
    steady_velocity.push_back(ship->GetSteadyVelocity());
    pert_velocity.push_back(ship->GetLinearVelocityPert());

  }

  // -------------------------------------------------------
  // Output plot
  // -------------------------------------------------------

  matplotlibcpp::subplot(2, 1, 1);
  PlotResults(vtime, position_body, "body", "position", "-");

  matplotlibcpp::subplot(2, 1, 2);
  PlotResults(vtime, velocity_body, "body", "velocity", "-");
  PlotResults(vtime, steady_velocity, "mean", "velocity", "--");

  matplotlibcpp::show();

  PlotResults(vtime, pert_velocity, "perturbation", "velocity");
  matplotlibcpp::show();

  // --------------------------------------------------------
  // Write CSV
  // --------------------------------------------------------

  unsigned int nt = vtime.size();
  std::string sep = ";";

  std::ofstream csvfile;
  csvfile.open("radiation_no_forward_speed.csv");

  csvfile << "time" << sep;
  csvfile << "X" << sep << "Y" << sep << "Z" << sep;
  csvfile << "VX" << sep << "VY" << sep << "VZ" << sep;
  csvfile << "Xs" << sep << "Ys" << sep << "Zs" << sep;
  csvfile << "Xp" << sep << "Yp" << sep << "Zp";
  csvfile << "\n";

  for (unsigned int i = 0; i < nt; i++) {
    csvfile << vtime[i] << sep;
    csvfile << position_body[i].x() << sep << position_body[i].y() << sep << position_body[i].z() << sep;
    csvfile << velocity_body[i].x() << sep << velocity_body[i].y() << sep << velocity_body[i].z() << sep;
    csvfile << steady_velocity[i].x() << sep << steady_velocity[i].y() << sep << steady_velocity[i].z() << sep;
    csvfile << pert_velocity[i].x() << sep << pert_velocity[i].y() << sep << pert_velocity[i].z();
    csvfile << "\n";
  }

  csvfile.close();

}