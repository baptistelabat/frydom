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
#include "frydom/logging/FrEventLogger.h"


using namespace frydom;


int main(int argc, char *argv[]) {

  std::cout << " ===================================================== \n"
               " Benchmark test : Nonlinear hydrostatics on a box \n"
               " ===================================================== \n" << std::endl;

  // -- System

  FrOffshoreSystem system("test_HS_equilibrium");

  auto Ocean = system.GetEnvironment()->GetOcean();
  Ocean->SetDensity(1023.);

  // -- Body
  auto body = system.NewBody("body");
  std::string meshFilename;

  enum bench_cases {
    box, DTMB, platform
  };

  bench_cases featuredCase = platform;

  switch (featuredCase) {
    case box: {

      double L, B, H, c;
      L = H = B = 5.;
      c = 0.5;
//    L = 8; B = 4; H = 2; c = 0.5;

      auto mass = L * H * B * c * system.GetEnvironment()->GetFluidDensity(WATER);
      makeItBox(body, L, B, H, mass);

      body->TranslateInWorld(0, 0, 0.75 * 2.5, NWU);

      double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
      auto inertia = body->GetInertiaTensor();
      inertia.GetInertiaCoeffsAtCOG(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);
      body->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, Position(-2.5, -2.5, -2.5), NWU));

      event_logger::info("main", "test_HS_equilibrium", "box mass : {}", mass);

      meshFilename = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/box/box_385_t.obj"});
//    meshFilename = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/box/box_385.obj"});
      break;
    }
    case DTMB: {
      body->SetInertiaTensor(FrInertiaTensor(86.0, 1.98, 53.88, 49.99, 0., 0., 0., Position(0., 0., 0.03), NWU));
      meshFilename = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/DTMB5512/DTMB5512.obj"});
      break;
    }
    case platform: {
      body->SetInertiaTensor(
          FrInertiaTensor(3.22114e7, 2.4e11, 2.3e11, 2e12, 0., 0., 0., Position(0.22, 0.22, 2.92), NWU));
      meshFilename = FrFileSystem::join(
          {system.config_file().GetDataFolder(), "ce/platform/mesh_Platform_GVA7500_Sym.obj"});
      body->TranslateInWorld(0, 0, 10, NWU);
      break;
    }
  }

  // At initial state
//  auto HydroMesh = make_hydro_mesh("hydroMesh", body, meshFilename, FrFrame(), FrHydroMesh::ClippingSupport::PLANESURFACE);
//  HydroMesh->Initialize();
//  HydroMesh->Update(0.);
//  FrHydrostaticsProperties hsp(system.GetEnvironment()->GetFluidDensity(WATER),
//                               system.GetGravityAcceleration(),
//                               HydroMesh->GetClippedMesh(),
//                               Position(), Position(), NWU);
//  hsp.Process();
//  event_logger::info("main", "test_HS_equilibrium", hsp.GetReport());
//  HydroMesh->GetClippedMesh().Write("Initial_Clipped_Mesh.obj");

  // At equilibrium
  event_logger::info("main", "test_HS_equilibrium", "Body COG position : {}", body->GetCOGPositionInWorld(NWU));

  auto staticEquilibrium = FrHydroStaticEquilibrium(body, meshFilename, FrFrame());
  staticEquilibrium.SetLinearRelaxation(1.);

  staticEquilibrium.Solve(body->GetInertiaTensor());

//  auto staticEquilibrium = solve_hydrostatic_equilibrium(body, meshFilename, FrFrame(), body->GetInertiaTensor());

  event_logger::info("main", "test_HS_equilibrium", "Body frame : {}", body->GetFrame());

  staticEquilibrium.GetHydroMesh()->GetClippedMesh().Write("Clipped_Mesh.obj");

  event_logger::info("main", "test_HS_equilibrium",
                     staticEquilibrium.GetReport(body->GetInertiaTensor().GetCOGPosition(NWU), {}, NWU));

}