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

int main(int argc, char *argv[]) {

  std::cout << " ===================================================== \n"
               " Benchmark test : Sphere Decay \n"
               " ===================================================== " << std::endl;

  // -- System

  FrOffshoreSystem system("Sphere_Decay");

  auto Ocean = system.GetEnvironment()->GetOcean();
  Ocean->SetDensity(1000);

  // -- Body

  auto body = system.NewBody("sphere");

  Position COGPosition(0., 0., -2.);

  body->SetPosition(Position(0., 0., 0.), NWU);

  // -- Inertia

  double mass = 2.618E5;

  double Ixx = 1.690E6;
  double Iyy = 1.690E6;
  double Izz = 2.606E6;

  FrInertiaTensor InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGPosition, NWU);

  body->SetInertiaTensor(InertiaTensor);

  body->GetDOFMask()->SetLock_X(true);
  body->GetDOFMask()->SetLock_Y(true);
  body->GetDOFMask()->SetLock_Rx(true);
  body->GetDOFMask()->SetLock_Ry(true);
  body->GetDOFMask()->SetLock_Rz(true);

  // -- Hydrodynamics

  //auto hdb = make_hydrodynamic_database(resources_path.resolve("sphere_hdb.h5").path());
  auto sphere_HDB = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/sphere/sphere_hdb.h5"});
  auto hdb = make_hydrodynamic_database(sphere_HDB);

  auto eqFrame = make_equilibrium_frame("EqFrame", &system, body);


  hdb->Map(0, body.get(), eqFrame);

  // -- Linear hydrostatics

  auto forceHst = make_linear_hydrostatic_force("linear_hydrostatic", body, hdb);

  // Nonlinear hydrostatics
  //auto bodyMesh = make_hydro_mesh(body,"Sphere_10000_faces.obj",FrFrame(),FrHydroMesh::ClippingSupport::WAVESURFACE);
  //bodyMesh->GetInitialMesh().Write("Mesh_Initial.obj");

  //auto forceHst = make_nonlinear_hydrostatic_force(body,bodyMesh);

  // -- Radiation

  auto radiationModel = make_radiation_convolution_model("radiation_convolution", &system, hdb);
  radiationModel->SetImpulseResponseSize(body.get(), 6., 0.01);

  // -- Simulation

  auto dt = 0.01;

  system.SetTimeStep(dt);
  system.Initialize();

  // Decay test initial position.
  body->SetPosition(Position(0., 0., 1.), NWU);

  auto time = 0.;

  clock_t begin = clock();

  while (time < 20.) {
    time += dt;
    system.AdvanceTo(time);
    std::cout << "time : " << time << std::endl;
  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Elapsed cpu time in seconds : " << elapsed_secs << std::endl;
  std::cout << "============================== End ===================================== " << std::endl;

} // end namespace frydom
