//
// Created by camille on 10/04/19.
//

#include "frydom/frydom.h"
#include <cppfs/FilePath.h>

using namespace frydom;

int main(int argc, char *argv[]) {

  std::cout << " ========================== Test Cylinder interaction ================== " << std::endl;

  // System

  FrOffshoreSystem system("test_cylinder_interaction");

  // Environment

  auto ocean = system.GetEnvironment()->GetOcean();
  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWaveHeight(0.001);
  waveField->SetWavePeriod(M_PI / 4.);
  waveField->SetDirection(0., DEG, NWU, GOTO);

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 15., 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(true);

  // Bodies

  auto cyl1 = system.NewBody("cylinder_1");
  cyl1->SetPosition(Position(0., -0.4, 0.), NWU);

  auto cyl2 = system.NewBody("cylinder_2");
  cyl2->SetPosition(Position(0., +0.4, 0.), NWU);

  double mass = 12.88;
  FrInertiaTensor InertiaTensor(mass, 1., 1., 1., 0., 0., 0., Position(0., 0., 0.), NWU);

  cyl1->SetInertiaTensor(InertiaTensor);
  cyl2->SetInertiaTensor(InertiaTensor);

  // DOFMask

  cyl1->GetDOFMask()->MakeItLocked();
  cyl1->GetDOFMask()->SetLock_Z(false);

  cyl2->GetDOFMask()->MakeItLocked();
  cyl2->GetDOFMask()->SetLock_Z(false);

  // Hydrodynamic

  auto cylinder_hdb = FrFileSystem::join(
      {system.config_file().GetDataFolder(), "ce/CylinderInteraction/CylinderInteraction.hdb5"});
  auto hdb = make_hydrodynamic_database(cylinder_hdb);

  auto eqFrame1 = make_equilibrium_frame("eq_frame_cylinder_1", &system, cyl1);
  auto eqFrame2 = make_equilibrium_frame("eq_frame_cylinder_2", &system, cyl2);

  hdb->Map(0, cyl1.get(), eqFrame1);
  hdb->Map(1, cyl2.get(), eqFrame2);

  auto radiationModel = make_radiation_convolution_model("HDB", &system, hdb);
  radiationModel->SetImpulseResponseSize(cyl1.get(), 8., 0.01);
  radiationModel->SetImpulseResponseSize(cyl2.get(), 8., 0.01);

  // Hydrostatic

  auto forceHst1 = make_linear_hydrostatic_force("linear_hydrostatics", cyl1, hdb);
  auto forceHst2 = make_linear_hydrostatic_force("linear_hydrostatics", cyl2, hdb);

  // Excitation

  auto excitation1 = make_linear_excitation_force("linear_excitation", cyl1, hdb);
  auto excitation2 = make_linear_excitation_force("linear_excitation", cyl2, hdb);

  // Simulation

  auto dt = 0.005;

  system.SetTimeStep(dt);

  std::cout << "Initialize ..." << std::endl;

  system.Initialize();

  std::cout << "Run simulation ..." << std::endl;

  double time = 0.;

  while (time < 50.) {

    time += dt;

    system.AdvanceTo(time);

    std::cout << "Time : " << time << " s" << std::endl;

    std::cout << "Position cyl1 : " << cyl1->GetPosition(NWU).GetX() << ";"
              << cyl1->GetPosition(NWU).GetY() << ";"
              << cyl1->GetPosition(NWU).GetZ() << std::endl;

    std::cout << "Position cyl2 : " << cyl2->GetPosition(NWU).GetX() << ";"
              << cyl2->GetPosition(NWU).GetY() << ";"
              << cyl2->GetPosition(NWU).GetZ() << std::endl;

  }

  std::cout << "======================================= End ============================= " << std::endl;


}
