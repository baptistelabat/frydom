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
using namespace mathutils;


void ValidationResults(const std::vector<double> vtime, const std::vector<double> heave,
                       const int iperiod, const int isteepness) {

  FrHDF5Reader db("bench_sphere_regular.h5");

  auto path = "T" + std::to_string(iperiod) + "/H" + std::to_string(isteepness);

  auto rao_bench = db.ReadDouble(path + "/rao");
  auto wave_height = db.ReadDouble(path + "/wave_height");
  auto period = db.ReadDouble(path + "/period");
  auto steepness = db.ReadDouble(path + "/steepness");

  int it = 0;
  while (vtime[it] < 100.) {
    it += 1;
  }

  auto motion = 0.;

  for (int i = it; i < vtime.size(); i++) {
    motion = std::max(motion, heave[i]);
  }

  auto rao = motion / (0.5 * wave_height);
  auto err_rel = std::abs(rao - rao_bench) / rao_bench;

  // Print results
  std::cout << "----------------------------------------" << std::endl;
  std::cout << " T = " << period << " , H = " << wave_height << ", Steepness = " << steepness << std::endl;
  std::cout << " RAO = " << rao << " bench : " << rao_bench << "error rel. : " << err_rel << std::endl;

  // Plot Results
  matplotlibcpp::named_plot("results", vtime, heave);
  matplotlibcpp::xlabel("time (s)");
  matplotlibcpp::ylabel("heave motion (m)");
  matplotlibcpp::xlim(0., 200.);
  matplotlibcpp::grid(true);
  matplotlibcpp::legend();
  matplotlibcpp::show();
}

std::vector<double> ReadParam(const std::string dbfile, const int iperiod, const int isteepness) {

  auto path = "T" + std::to_string(iperiod) + "/H" + std::to_string(isteepness);

  std::vector<double> param(2);

  FrHDF5Reader db(dbfile);

  param[0] = db.ReadDouble(path + "/period");
  param[1] = db.ReadDouble(path + "/wave_height");

  auto steepness = db.ReadDouble(path + "/steepness");

  std::cout << "Regular wave : T = " << param[0] << " s , Wave Height = "
            << param[1] << " m " << " , steepness = " << steepness << std::endl;

  return param;

}


int main(int argc, char *argv[]) {


  std::cout << " -------------------------------------------- \n "
               " Run Sphere Regular Wave free motion Benchmark \n "
               " -------------------------------------------- " << std::endl;

  // System
  FrOffshoreSystem system;

  // Environment
  system.GetEnvironment()->SetWaterDensity(1000.);
  system.GetEnvironment()->GetFreeSurface()->SetGrid(-20., 20., 5.);

  auto iperiod = 1;
  auto isteepness = 1;
  if (argv[1]) { iperiod = atoi(argv[1]); }
  if (argv[2]) { isteepness = atoi(argv[2]); }

  // Set Free surface & wave field

  auto param = ReadParam("bench_sphere_regular.h5", iperiod, isteepness);

  auto freeSurface = system.GetEnvironment()->GetFreeSurface();
  freeSurface->SetLinearWaveField(LINEAR_REGULAR);
  auto waveField = freeSurface->GetLinearWaveField();
  waveField->SetRegularWaveHeight(param[1]);
  waveField->SetRegularWavePeriod(param[0]);
  waveField->SetMeanWaveDirection(0.);
  waveField->GetSteadyElevation(0, 0);
  // waveField->GetWaveRamp()->SetDuration(5.);
  // waveField->GetWaveRamp()->SetIncrease();

  // Body
  auto sphere = std::make_shared<FrShip>();
  sphere->SetName("sphere");
  sphere->SetHydroMesh("sphere.obj", true);
  sphere->SetEquilibriumFrame(BodyFixed, chrono::ChVector<>());
  sphere->SetInertiaXX(chrono::ChVector<double>(1.690e6, 1.690e6, 2.606e6));
  sphere->SetMass(2.618e5);
  sphere->SetCOG(chrono::ChVector<double>(0., 0., -2.));

  system.AddBody(sphere);

  sphere->Set3DOF_ON(chrono::ChVector<>(1, 0, 0)); // FIXME : need to be adapted to select only one dof

  // Hydrostatic
  auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
  auto hstStiffness = hstForce->GetStiffnessMatrix();
  double K33 = 7.6947e5;
  double K44 = 5.1263e6;
  hstStiffness->SetDiagonal(K33, K44, K44);
  sphere->AddForce(hstForce);

  // Hydrodynamic load
  FrHydroDB HDB = LoadHDB5("sphere_hdb.h5");

  auto hydroMapper = HDB.GetMapper();
  system.SetHydroMapper(hydroMapper);
  hydroMapper->Map(sphere, 0);

  auto radModel = std::make_shared<FrRadiationConvolutionModel>(&HDB, &system);
  radModel->AddRadiationForceToHydroBody(sphere);

  // Excitation force
  auto waveProbe = waveField->NewWaveProbe(0, 0);
  waveProbe->Initialize();

  auto excForce = std::make_shared<FrLinearExcitationForce>();
  sphere->AddForce(excForce);
  excForce->SetWaveProbe(waveProbe);

  // Numerical scheme
  system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);

  double time;
  double dt = 0.005;

  std::vector<double> heave;
  std::vector<double> vtime;
  chrono::ChVector<double> position;

  system.SetStep(dt);
  system.Initialize();

  auto icase = iperiod;

  heave.clear();
  vtime.clear();

  time = -dt;

  while (time < 200.) {

    time += dt;

    system.DoStepDynamics(dt);

    position = sphere->GetPosition() - sphere->GetRelCOG();

    heave.push_back(position[2]);
    vtime.push_back(time);

  }

  ValidationResults(vtime, heave, iperiod, isteepness);
}