//
// Created by camille on 12/04/19.
//
#include <cppfs/fs.h>
#include <frydom/logging/FrTypeNames.h>
#include "frydom/frydom.h"

using namespace frydom;

// --------------------------------------------------------------------
// Buoyancy force
// --------------------------------------------------------------------

namespace frydom {
  class FrDiffBuoyancyForce : public FrForce {

   public:

    FrDiffBuoyancyForce(const std::string &name, FrBody *body) :
        FrForce(name, TypeToString(this), body) {};

    void Compute(double time) override {

      double grav = GetSystem()->GetEnvironment()->GetGravityAcceleration();
      double massPTO = 1.e5;
      double mg = massPTO * grav;

      SetForceInWorldAtCOG(Force(0., 0., mg), NWU);

    }

    void StepFinalize() override {}

  };

  TYPE_TO_STRING(FrDiffBuoyancyForce, "DiffBuoyancyForce")
}
// --------------------------------------------------------------------
// Main
// --------------------------------------------------------------------

int main(int argc, char *argv[]) {

  std::cout << " ====================================== Demo Langlee F3OF ========================== " << std::endl;

  // System
  FrOffshoreSystem system("demo_Langlee");

  //system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
  //system.SetSolverVerbose(true);

  // Environment

  auto ocean = system.GetEnvironment()->GetOcean();
  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWaveHeight(0.);
  waveField->SetWavePeriod(10.);
  waveField->SetDirection(0., DEG, NWU, GOTO);

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 15., 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(false);

  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-14., 14, 2, -14, 14, 2);

  // Bodies

  auto barge = system.NewBody("barge");
  auto bargeMesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/Langlee/FullBarge_e125000.obj"});
  barge->AddMeshAsset(bargeMesh);
  barge->SetPosition(Position(0., 0., -8.5), NWU);

  double mass_b = 1.089825e6;
  double Iy_b = 7.63e7;
  FrInertiaTensor InertiaTensor(mass_b, Iy_b, Iy_b, Iy_b, 0., 0., 0., Position(0., 0., 0.), NWU);
  barge->SetInertiaTensor(InertiaTensor);

  barge->GetDOFMask()->MakeItLocked();

  auto node_1b = barge->NewNode("node_1b");
  node_1b->SetPositionInBody(Position(-12.5, 0., 0.), NWU);
  node_1b->RotateAroundXInBody(-90. * DEG2RAD, NWU);

  auto node_2b = barge->NewNode("node_2b");
  node_2b->SetPositionInBody(Position(12.5, 0., 0.), NWU);
  node_2b->RotateAroundXInBody(-90. * DEG2RAD, NWU);

  // Flap1

  // -- Position
  auto flap1 = system.NewBody("flap1");
  auto flap1Mesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/Langlee/FullFlap1.obj"});
  flap1->AddMeshAsset(flap1Mesh);
  flap1->SetPosition(Position(-12.5, 0., -8.5), NWU);

  // -- Inertia
  double mass_f1 = 1.7925e5; //;2.8874e5
  double Iy_f1 = 1.3e6; //1.3e6;
  FrFrame COGFrame_flap(Position(0., 0., 3.5), FrRotation(), NWU);
  FrInertiaTensor InertiaTensor_flap(mass_f1, Iy_f1, Iy_f1, Iy_f1, 0., 0., 0., Position(0., 0., 3.5), NWU);

  flap1->SetInertiaTensor(InertiaTensor_flap);

  // -- Link
  auto node_1f = flap1->NewNode("node_1f");
  node_1f->SetPositionInBody(Position(0., 0., 0), NWU);
  node_1f->RotateAroundXInBody(-90. * DEG2RAD, NWU);

  auto rev1 = make_revolute_link("barge-flap1_revolute", &system, node_1f, node_1b);

  // -- PTO
  //auto forcePTO_1 = make_linear_damping_force(flap1, WATER, false);
  //forcePTO_1->SetDiagonalRotationDamping(0., 1.e6, 0.);

  // -- Viscous damping
  //auto morisonModel_1 = make_morison_model(flap1.get());
  //morisonModel_1->AddElement(Position(0., 0., 0.), Position(0., 0., 11.), 2., 0., 8., 0., 10);
  //auto morisonForce_1 = make_morison_force(morisonModel_1, flap1);

  // Flap2

  // -- Position
  auto flap2 = system.NewBody("flap2");
  auto flap2Mesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/Langlee/FullFlap2.obj"});
  flap2->AddMeshAsset(flap2Mesh);
  flap2->SetPosition(Position(12.5, 0., -8.5), NWU);

  flap2->SetInertiaTensor(InertiaTensor_flap);

  // -- Link
  auto node_2f = flap2->NewNode("node_2f");
  node_2f->SetPositionInBody(Position(0., 0., 0), NWU);
  node_2f->RotateAroundXInBody(-90. * DEG2RAD, NWU);

  auto rev2 = make_revolute_link("barge-flap2_revolute", &system, node_2f, node_2b);

  // -- PTO
  //auto forcePTO_2 = make_linear_damping_force(flap2, WATER, false);
  //forcePTO_2->SetDiagonalRotationDamping(0., 1.e6, 0.);

  // -- Viscous damping
  //auto morisonModel_2 = make_morison_model(flap2.get());
  //morisonModel_2->AddElement(Position(0., 0., 0.), Position(0., 0., 11.), 2., 0., 8., 10);
  //auto morisonForce_2 = make_morison_force(morisonModel_2, flap2);

  // Hydrodynamic

  auto bargeHDB = FrFileSystem::join(
      {system.config_file().GetDataFolder(), "ce/Langlee/Langlee_draft8_5_filtered_t50.hdb5"});
  auto hdb = make_hydrodynamic_database(bargeHDB);

  auto eqFrame0 = make_equilibrium_frame("eqFrame0", &system, barge);
  auto eqFrame1 = make_equilibrium_frame("eqFrame1", &system, flap1);
  auto eqFrame2 = make_equilibrium_frame("eqFrame2", &system, flap2);

  hdb->Map(0, flap1.get(), eqFrame1);
  hdb->Map(1, flap2.get(), eqFrame2);
  hdb->Map(2, barge.get(), eqFrame0);

  auto radiationModel = make_radiation_convolution_model("radiation_convolution", &system, hdb);
  radiationModel->SetImpulseResponseSize(flap1.get(), 80., 0.01);
  radiationModel->SetImpulseResponseSize(flap2.get(), 80., 0.01);
  radiationModel->SetImpulseResponseSize(barge.get(), 80., 0.01);

  // Hydrostatic

  // -- Linear
  auto forceHst1 = make_linear_hydrostatic_force("linear_hydrostatic_flap1", flap1, hdb);
  auto forceHst2 = make_linear_hydrostatic_force("linear_hydrostatic_flap2", flap2, hdb);

  auto diffBuoyForce1 = std::make_shared<FrDiffBuoyancyForce>("diffBuoyForce1", flap1.get());
  flap1->AddExternalForce(diffBuoyForce1);

  auto diffBuoyForce2 = std::make_shared<FrDiffBuoyancyForce>("diffBuoyForce2", flap2.get());
  flap2->AddExternalForce(diffBuoyForce2);

  // -- Nonlinear
  //auto flap1Mesh = make_hydro_mesh(flap1, system.GetDataPath("FullFlap_sym_wsep_draft8_5_fillet.obj"),
  //        FrFrame(), FrHydroMesh::ClippingSupport::PLANESURFACE);
  //auto forceHst1 = make_nonlinear_hydrostatic_force(flap1, flap1Mesh);

  //auto flap2Mesh = make_hydro_mesh(flap2, system.GetDataPath("FullFlap_sym_wsep_draft8_5_fillet.obj"),
  //        FrFrame(), FrHydroMesh::ClippingSupport::PLANESURFACE);
  //auto forceHst2 = make_nonlinear_hydrostatic_force(flap2, flap2Mesh);

  //flap1Mesh->GetInitialMesh().Write("HydroMesh_Flap1_Initial.obj");
  //flap2Mesh->GetInitialMesh().Write("HydroMesh_Flap2_Initial.obj");

  // Excitation

  //auto excitation1 = make_linear_excitation_force(hdb, flap1);
  //auto excitation2 = make_linear_excitation_force(hdb, flap2);

  // Simulation

  auto dt = 0.01;

  system.SetTimeStep(dt);

  system.Initialize();

  flap1->Rotate(FrRotation(Direction(0, 1, 0), 10. * DEG2RAD, NWU));

  bool is_irrlicht = true;

  if (is_irrlicht) {
    system.RunInViewer(100, 50, false);
  } else {
    auto time = 0.;
    while (time < 10.) {
      time += dt;
      system.AdvanceTo(time);
      std::cout << "Time : " << time << " s" << std::endl;
    }
  }

  std::cout << " ==================================== End ========================== " << std::endl;

  return 0;

}

