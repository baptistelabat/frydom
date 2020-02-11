//
// Created by camille on 09/05/19.
//

#include "frydom/frydom.h"

using namespace frydom;

// -----------------------------------------------------------------------
// FOSWEC Model definition
// -----------------------------------------------------------------------

void DemoModel(FrOffshoreSystem &system, bool flap1_fixed, bool flap2_fixed, double initial_angle) {

  // System
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-1, 1, 2, -1, 1, 2);

  // PLatform

  auto platform = system.NewBody("platform");
  auto platformMesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/FOSWEC/FullPlatform.obj"});
  platform->AddMeshAsset(platformMesh);

  FrInertiaTensor inertia_b(153.8, 37.88, 29.63, 1., 0., 0., 0., Position(0., 0., 0.460), NWU);
  platform->SetInertiaTensor(inertia_b);

  platform->GetDOFMask()->MakeItLocked();

  auto node_1b = platform->NewNode("node_1b");
  node_1b->SetPositionInBody(Position(-0.65, 0., -0.5008), NWU);
  node_1b->RotateAroundXInBody(-90. * DEG2RAD, NWU);

  auto node_2b = platform->NewNode("node_2b");
  node_2b->SetPositionInBody(Position(0.65, 0., -0.5008), NWU);
  node_2b->RotateAroundXInBody(-90. * DEG2RAD, NWU);

  // Flap1

  auto flap1 = system.NewBody("flap1");
  auto flap1Mesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/FOSWEC/FullFlap_mesh.obj"});
  flap1->AddMeshAsset(flap1Mesh);
  flap1->SetPosition(Position(-0.65, 0., -0.29), NWU);

  FrInertiaTensor inertia_f1(23.1, 1.42, 1.19, 1.99, 0., 0., 0., Position(0., 0., 0.), NWU);
  flap1->SetInertiaTensor(inertia_f1);

  auto node_1f = flap1->NewNode("node_1f");
  node_1f->SetPositionInBody(Position(0., 0., -0.2108), NWU);
  node_1f->RotateAroundXInBody(-90. * DEG2RAD, NWU);

  if (flap1_fixed) {
    auto rev1 = make_fixed_link("fixed1", &system, node_1f, node_1b);
  } else {
    auto rev1 = make_revolute_link("rev1", &system, node_1f, node_1b);
  }

  if (std::abs(initial_angle) > DBL_EPSILON) {
    flap1->RotateAroundPointInBody(FrRotation(Direction(0., 1., 0.), initial_angle * DEG2RAD, NWU),
                                   node_1f->GetNodePositionInBody(NWU), NWU);
  }
  // Flap2

  auto flap2 = system.NewBody("flap2");
  flap2->AddMeshAsset(flap1Mesh);
  flap2->SetPosition(Position(0.65, 0., -0.29), NWU);

  FrInertiaTensor inertia_f2(23.1, 1.42, 1.19, 1.99, 0., 0., 0., Position(0., 0., 0.), NWU);
  flap2->SetInertiaTensor(inertia_f2);

  auto node_2f = flap2->NewNode("node_2f");
  node_2f->SetPositionInBody(Position(0., 0., -0.2108), NWU);
  node_2f->RotateAroundXInBody(-90. * DEG2RAD, NWU);

  if (flap2_fixed) {
    auto rev2 = make_fixed_link("fixed2", &system, node_2f, node_2b);
  } else {
    auto rev2 = make_revolute_link("rev2", &system, node_2f, node_2b);
  }

  // Hydrodynamic and radiation model

  auto FOSWEC_HDB = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/FOSWEC/FOSWEC_phase2_filtered.hdb5"});
  auto hdb = make_hydrodynamic_database(FOSWEC_HDB);

  auto eqFrame0 = make_equilibrium_frame("eqFrame0", platform);
  auto eqFrame1 = make_equilibrium_frame("eqFrame1", flap1);
  auto eqFrame2 = make_equilibrium_frame("eqFrame2", flap2);

  hdb->Map(0, flap1.get(), eqFrame1);
  hdb->Map(1, flap2.get(), eqFrame2);
  hdb->Map(2, platform.get(), eqFrame0);

  auto radiationModel = make_radiation_convolution_model("radiation_convolution", &system, hdb);
  radiationModel->SetImpulseResponseSize(flap1.get(), 60., 0.01);
  radiationModel->SetImpulseResponseSize(flap2.get(), 60., 0.01);
  radiationModel->SetImpulseResponseSize(platform.get(), 60., 0.01);

  // Hydrostatic

  //--> Linear
  auto forceHst1 = make_linear_hydrostatic_force("linear_hydrostatic_flap1", flap1, hdb);
  auto forceHst2 = make_linear_hydrostatic_force("linear_hydrostatic_flap2", flap2, hdb);

  //--> Nonlinear
  //auto frameOffset1 = FrFrame(Position(0., 0., 0.), FrRotation(), NWU);
  //auto flapMesh1 = make_hydro_mesh(flap1, resources_path + "FullFlap1_fine.obj", frameOffset1,
  //                                 FrHydroMesh::ClippingSupport::PLANESURFACE);
  //auto forceHst1 = make_nonlinear_hydrostatic_force(flap1, flapMesh1);

  //auto frameOffset2 = FrFrame(Position(0., 0., 0.), FrRotation(), NWU);
  //auto flapMesh2 = make_hydro_mesh(flap2, resources_path + "FullFlap1_fine.obj", frameOffset2,
  //                                 FrHydroMesh::ClippingSupport::PLANESURFACE);
  //auto forceHst2 = make_nonlinear_hydrostatic_force(flap2, flapMesh2);

  // Excitation force

  //--> Linear
  auto excitation1 = make_linear_excitation_force("linear_excitation_flap1", flap1, hdb);
  auto excitation2 = make_linear_excitation_force("linear_excitation_flap2", flap2, hdb);

  // Viscous damping

  auto morisonModel_1 = make_morison_model(flap1.get());
  morisonModel_1->AddElement(Position(0., 0., -0.16), Position(0., 0., 0.42), 0.1, 0., 8., 0., 30);
  auto morisonForce_1 = make_morison_force("morison_force_flap1", flap1, morisonModel_1);

  auto morisonModel_2 = make_morison_model(flap2.get());
  morisonModel_2->AddElement(Position(0., 0., -0.16), Position(0., 0., 0.42), 0.1, 0., 8., 0., 30);
  auto morisonForce_2 = make_morison_force("morison_force_flap2", flap2, morisonModel_2);
}


// ------------------------------------------------------------------------
// Definition of the environment conditions
// ------------------------------------------------------------------------

static void SetEnvironment(FrOffshoreSystem &system, double period, double amplitude) {

  auto waveField = system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWaveHeight(amplitude);
  waveField->SetWavePeriod(period);
  waveField->SetDirection(0., DEG, NWU, GOTO);

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 10., 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(true);
}

// ------------------------------------------------------------------------------
// Simulation launcher
// ------------------------------------------------------------------------------

void Run(FrOffshoreSystem &system) {

  double dt = 0.01;

  system.SetTimeStep(dt);
  system.Initialize();

  bool use_irrlicht = true;

  if (use_irrlicht) {

    system.RunInViewer(0., 5, false);

  } else {

    double time = 0.;
    while (time < 35.) {

      time += dt;

      system.AdvanceTo(time);

      std::cout << "Time : " << time << " s" << std::endl;
    }
  }

}

// ----------------------------------------------------------------------
// Demo configuration 1 regular wave
// ----------------------------------------------------------------------

void DemoConfig1Reg() {

  // System
  FrOffshoreSystem system("FOSWEC_config1Reg");
  // Environment
  SetEnvironment(system, 3.333, 0.0225);
  // Configuration
  DemoModel(system, false, true, 0.);
  // Run
  Run(system);
}

// -----------------------------------------------------------------------
// Demo configuration 2 regular wave
// -----------------------------------------------------------------------

void DemoConfig2Reg() {

  // System
  FrOffshoreSystem system("FOSWEC_config2Reg");
  // Environment
  SetEnvironment(system, 3.333, 0.0225);
  // Configuration
  DemoModel(system, false, false, 0.);
  // Run
  Run(system);
}

// ------------------------------------------------------------------------
// Demo Wave excitation
// ------------------------------------------------------------------------

void DemoWaveExcitation() {

  // System
  FrOffshoreSystem system("FOSWEC_waveExcitation");
  // Environment
  SetEnvironment(system, 3.333, 0.0225);
  // Configuration
  DemoModel(system, false, false, 0.);
  // Run
  Run(system);
}

// --------------------------------------------------------------------------
// Demo Flap decay test
// --------------------------------------------------------------------------

void DemoFlapDecay() {

  // System
  FrOffshoreSystem system("FOSWEC_flapDecay");
  // Environment
  SetEnvironment(system, 10., 0.);
  // Configuration
  DemoModel(system, false, false, 10.);
  // Run
  Run(system);
}

int main(int argc, char *argv[]) {

  std::cout << " ============================================== Demo FOSWEC ============================= "
            << std::endl;

  std::cout << "Flap Decay..." << std::endl;
  DemoFlapDecay();

//  std::cout << "Wave excitation..." << std::endl;
//  DemoWaveExcitation();

//  std::cout << "Configuration 1 regular wave..." << std::endl;
//  DemoConfig1Reg();

//  std::cout << "Configuration 2 regular wave..." << std::endl;
//  DemoConfig2Reg();

  std::cout << " ================================================= End ================================ " << std::endl;

  return 0;
}