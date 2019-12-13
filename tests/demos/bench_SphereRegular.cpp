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


void ValidationResults(const std::vector<double> vtime, const std::vector<double> heave, const std::string dbfile,
                       const int iperiod, const int isteepness) {

  FrHDF5Reader db(dbfile);

  auto path = "T" + std::to_string(iperiod) + "/H" + std::to_string(isteepness);

  auto rao_bench = db.ReadDouble(path + "/rao");
  auto wave_height = db.ReadDouble(path + "/wave_height");
  auto period = db.ReadDouble(path + "/period");
  auto steepness = db.ReadDouble(path + "/steepness");

  int it = 0;
  while (vtime[it] < 100.) {
    it += 1;
  }

  auto motionMax = -999.;
  for (int i = it; i < vtime.size(); i++) {
    motionMax = std::max(motionMax, heave[i]);
  }

  auto motionMin = 999.;
  for (int i = it; i < vtime.size(); i++) {
    motionMin = std::min(motionMin, heave[i]);
  }

//  auto rao = motion / (0.5 * wave_height);
  auto rao = ((motionMax - motionMin) * 0.5) / (0.5 * wave_height);
  auto err_rel = std::abs(rao - rao_bench) / rao_bench;

  // Print results
  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << " T = " << period << " , H = " << wave_height << ", Steepness = " << steepness << std::endl;
  std::cout << " RAO = " << rao << " Bench : " << rao_bench << " Error rel. : " << err_rel << std::endl;

  // Print to file
  std::ofstream myfile;
  myfile.open("sphere_regular.csv", std::ios::out | std::ios::app);

  myfile << period << ";" << wave_height << ";" << steepness << ";"
         << rao << ";" << ";" << rao_bench << ";" << err_rel << std::endl;

  myfile.close();

}

std::vector<double> ReadParam(const std::string dbfile, const int iperiod, const int isteepness) {

  auto path = "T" + std::to_string(iperiod) + "/H" + std::to_string(isteepness);

  std::vector<double> param(2);

  FrHDF5Reader db(dbfile);

  param[0] = db.ReadDouble(path + "/period");
  param[1] = db.ReadDouble(path + "/wave_height");

  auto steepness = db.ReadDouble(path + "/steepness");

  std::cout << "Regular wave T = " << param[0] << " s, Wave Height = "
            << param[1] << " m " << "steepness = " << steepness << std::endl;

  return param;
}

int main(int argc, char *argv[]) {

  std::cout << " ==================================================== \n"
               " Benchmark test : Heave motion in regular waves \n"
               " ==================================================== " << std::endl;

  // -- Input

  int iPeriod = 0;
  int iSteepness = 0;

  if (argv[1]) { iPeriod = atoi(argv[1]); }
  if (argv[2]) { iSteepness = atoi(argv[2]); }

  // -- System

  FrOffshoreSystem system("Sphere_RW");

  // -- Ocean

  auto ocean = system.GetEnvironment()->GetOcean();
  ocean->SetInfiniteDepth();
  ocean->SetDensity(1000.);

  // -- Wave field

  auto data = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/sphere/bench_sphere_regular.h5"});
  auto param = ReadParam(data, iPeriod, iSteepness);

  double waveHeight = 0.5 * param[1];
  double wavePeriod = param[0];

  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWaveHeight(waveHeight);
  waveField->SetWavePeriod(wavePeriod);
  waveField->SetDirection(NORTH(NWU), NWU, GOTO);

  // -- Body

  auto body = system.NewBody("Sphere");

  Position COGPosition(0., 0., -2.);

  body->SetPosition(Position(0., 0., 0.), NWU);

  body->GetDOFMask()->SetLock_X(true);
  body->GetDOFMask()->SetLock_Y(true);
  body->GetDOFMask()->SetLock_Rx(true);
  body->GetDOFMask()->SetLock_Ry(true);
  body->GetDOFMask()->SetLock_Rz(true);

  // -- Inertia

  double mass = 2.618E5; // Theoretical mass.
//  double mass = 2.61299E5; // Mass from mesh with 6200 faces.
//  double mass = 2.61488E5; // Mass from mesh with 10000 faces.

  double Ixx = 1.690E6;
  double Iyy = 1.690E6;
  double Izz = 2.606E6;

  FrInertiaTensor InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGPosition, NWU);

  body->SetInertiaTensor(InertiaTensor);

  // -- Hydrodynamics
  auto sphere_hdb = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/sphere/sphere_hdb.h5"});
  auto hdb = make_hydrodynamic_database(sphere_hdb);

  auto eqFrame = make_equilibrium_frame("EqFrame", &system, body);


  hdb->Map(0, body.get(), eqFrame);

  // -- Linear hydrostatics

  auto forceHst = make_linear_hydrostatic_force("linear_hydrostatic", body, hdb);

  // -- Radiation

  auto radiationModel = make_radiation_convolution_model("radiation_convolution", &system, hdb);
  radiationModel->SetImpulseResponseSize(body.get(), 6., 0.1);

  // -- Linear diffraction

//  auto diffractionForce = make_linear_diffraction_force("linear_diffraction", body, hdb);

  // -- Linear Froude-Krylov

//  auto LinFKForce = make_linear_froude_krylov_force("linear_froude_krylov", body, hdb);

  // -- Linear excitation

  auto excitationForce = make_linear_excitation_force("linear_excitation", body, hdb);

  // -- Hydrodynamic mesh

//  auto sphere_mesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/sphere/Sphere_6200_faces.obj"});
//  auto bodyMesh = make_hydro_mesh("sphere_mesh", body, sphere_mesh, FrFrame(), FrHydroMesh::ClippingSupport::WAVESURFACE);
//  bodyMesh->GetInitialMesh().Write("Mesh_Initial.obj");

  // -- Nonlinear hydrostatics

//  auto forceHst = make_nonlinear_hydrostatic_force("nonlinear_hydrostatic", body, bodyMesh);
//  forceHst->SetLogged(true);

  // -- Nonlinear Froude-Krylov

//  auto NonlinFKForce = make_nonlinear_froude_krylov_force("nonlinear_froude_krylov", body, bodyMesh);
//  NonlinFKForce->SetLogged(true);

  // -- Simulation

  auto dt = 0.005;

  system.SetTimeStep(dt);
  system.Initialize();

  auto time = -dt;

  std::vector<double> vtime;
  std::vector<double> heave;

  clock_t begin = clock();

  while (time < 200.) {
    time += dt;
    system.AdvanceTo(time);

    std::cout << "time : " << time << " s" << std::endl;

    heave.push_back(body->GetPosition(NWU).GetZ());
    vtime.push_back(time);
  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << elapsed_secs << std::endl;

  ValidationResults(vtime, heave, data, iPeriod, iSteepness);

  std::cout << " ================================= End ======================= " << std::endl;

}
