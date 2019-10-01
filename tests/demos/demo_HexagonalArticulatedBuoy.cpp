//
// Created by camille on 11/07/19.
//

#include "frydom/frydom.h"

using namespace frydom;

void SetUpEnvironment(FrOffshoreSystem *system) {
  // Environment

  auto ocean = system->GetEnvironment()->GetOcean();
  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWavePeriod(3.);
  waveField->SetWaveHeight(0.1); //0.1
  waveField->SetDirection(0., DEG, NWU, GOTO);

  system->GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 15., 1.);
  system->GetEnvironment()->GetTimeRamp()->SetActive(false);

  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-5., 10., 0.5, -5., 5., 0.5);
  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetUpdateStep(5);

  ocean->GetSeabed()->SetBathymetry(-7.5, NWU);
  ocean->GetSeabed()->GetSeabedGridAsset()->SetGrid(-10., 15., 0.5, -12., 12., 0.5);
//    ocean->ShowSeabed(false);

  auto current = ocean->GetCurrent()->GetFieldUniform();
  current->SetNorth(0.5, KNOT, GOTO);
}

std::shared_ptr<FrBody> CreateBody(FrOffshoreSystem *system, Position bodyPos, FrRotation bodyRot) {

  auto body = system->NewBody();

  auto asset = std::make_shared<FrTriangleMeshConnected>();
  asset->LoadWavefrontMesh(system->GetDataPath("cylinder_base_bar.obj"));
  asset->Rotate(bodyRot);
  body->AddMeshAsset(asset);

  body->SetColor(Yellow);

  body->SetPosition(bodyPos, NWU);
  FrInertiaTensor inertiaTensor1(
      805.033,
      318.66, 100.63, 318.66, 0., 0., 0.,
      FrFrame(Position(0., 0., 0.), bodyRot, NWU),
      Position(0., 0., -0.1),
      NWU);
  body->SetInertiaTensor(FrInertiaTensor(805.033, 318.66, 100.63, 318.66, 0., 0., 0.,
                                         FrFrame(Position(0., 0., 0.), bodyRot, NWU), Position(0., 0., -0.1), NWU));

  return body;
}

void LinkBodies(FrOffshoreSystem *system, std::vector<std::shared_ptr<FrBody>> bodyList) {

  auto node_1a = bodyList[0]->NewNode();
  node_1a->SetPositionInWorld(Position(-2.498, 1.444, 0.), NWU);
  auto node_1b = bodyList[1]->NewNode();
  node_1b->SetPositionInWorld(Position(-2.498, 1.444, 0.), NWU);
  auto link1 = make_spherical_link(node_1a, node_1b, system);

  auto node_2a = bodyList[1]->NewNode();
  node_2a->SetPositionInWorld(Position(0., 2.887, 0.), NWU);
  auto node_2b = bodyList[2]->NewNode();
  node_2b->SetPositionInWorld(Position(0., 2.887, 0.), NWU);
  auto link2 = make_spherical_link(node_2a, node_2b, system);

  auto node_3a = bodyList[2]->NewNode();
  node_3a->SetPositionInWorld(Position(2.498, 1.444, 0.), NWU);
  auto node_3b = bodyList[3]->NewNode();
  node_3b->SetPositionInWorld(Position(2.498, 1.444, 0.), NWU);
  auto link3 = make_spherical_link(node_3a, node_3b, system);

  auto node_4a = bodyList[3]->NewNode();
  node_4a->SetPositionInWorld(Position(2.498, -1.444, 0.), NWU);
  auto node_4b = bodyList[4]->NewNode();
  node_4b->SetPositionInWorld(Position(2.498, -1.444, 0.), NWU);
  auto link4 = make_spherical_link(node_4a, node_4b, system);

  auto node_5a = bodyList[4]->NewNode();
  node_5a->SetPositionInWorld(Position(0., -2.887, 0.), NWU);
  auto node_5b = bodyList[5]->NewNode();
  node_5b->SetPositionInWorld(Position(0., -2.887, 0.), NWU);
  auto link5 = make_spherical_link(node_5a, node_5b, system);

  auto node_6a = bodyList[5]->NewNode();
  node_6a->SetPositionInWorld(Position(-2.498, -1.444, 0.), NWU);
  auto node_6b = bodyList[0]->NewNode();
  node_6b->SetPositionInWorld(Position(-2.498, -1.444, 0.), NWU);
  auto link6 = make_spherical_link(node_6a, node_6b, system);

//    auto node_7a = bodyList[0]->NewNode();
//    node_7a->SetPositionInWorld(Position(-2.498, 0., 0.), NWU);
//    auto node_7b = system->GetWorldBody()->NewNode();
//    node_7b->SetPositionInWorld(Position(-2.498, 0., 0.), NWU);
//    auto link7 = make_prismatic_link(node_7a, node_7b, system);
//
//    auto node_8a = bodyList[3]->NewNode();
//    node_8a->SetPositionInWorld(Position(2.498, 0., 0.), NWU);
//    auto node_8b = system->GetWorldBody()->NewNode();
//    node_8b->SetPositionInWorld(Position(2.498, 0., 0.), NWU);
//    auto link8 = make_prismatic_link(node_8a, node_8b, system);

}

std::shared_ptr<FrHydroDB>
SetUpHydrodynamicModel(FrOffshoreSystem *system, std::vector<std::shared_ptr<FrBody>> bodyList) {

  auto hdb = make_hydrodynamic_database(system->GetDataPath("hexagonal_articulated_buoy.hdb5"));

  auto eqFrame0 = make_equilibrium_frame(bodyList[0], system);
  auto eqFrame1 = make_equilibrium_frame(bodyList[1], system);
  auto eqFrame2 = make_equilibrium_frame(bodyList[2], system);
  auto eqFrame3 = make_equilibrium_frame(bodyList[3], system);
  auto eqFrame4 = make_equilibrium_frame(bodyList[4], system);
  auto eqFrame5 = make_equilibrium_frame(bodyList[5], system);

  eqFrame0->SetPositionInWorld(Position(-2.498, 0., 0.), NWU);
  eqFrame1->SetPositionInWorld(Position(-1.25, 2.165, 0.), NWU);
  eqFrame2->SetPositionInWorld(Position(1.25, 2.165, 0.), NWU);
  eqFrame3->SetPositionInWorld(Position(2.498, 0., 0.), NWU);
  eqFrame4->SetPositionInWorld(Position(1.25, -2.165, 0.), NWU);
  eqFrame5->SetPositionInWorld(Position(-1.25, -2.165, 0.), NWU);

  hdb->Map(0, bodyList[0].get(), eqFrame0);
  hdb->Map(1, bodyList[1].get(), eqFrame1);
  hdb->Map(2, bodyList[2].get(), eqFrame2);
  hdb->Map(3, bodyList[3].get(), eqFrame3);
  hdb->Map(4, bodyList[4].get(), eqFrame4);
  hdb->Map(5, bodyList[5].get(), eqFrame5);

  auto radiationModel = make_radiation_convolution_model(hdb, system);
  radiationModel->SetImpulseResponseSize(bodyList[0].get(), 50., 0.02);
  radiationModel->SetImpulseResponseSize(bodyList[1].get(), 50., 0.02);
  radiationModel->SetImpulseResponseSize(bodyList[2].get(), 50., 0.02);
  radiationModel->SetImpulseResponseSize(bodyList[3].get(), 50., 0.02);
  radiationModel->SetImpulseResponseSize(bodyList[4].get(), 50., 0.02);
  radiationModel->SetImpulseResponseSize(bodyList[5].get(), 50., 0.02);

  return hdb;
}

void SetUpMorisonModel(FrOffshoreSystem *system, std::vector<std::shared_ptr<FrBody>> bodyList) {

  auto cd = 3.2;

  auto morisonModel1 = make_morison_model(bodyList[0].get());
  //morisonModel1->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
  morisonModel1->AddElement(Position(0., -1, 0.), Position(0., 1., 0.), 1., 0., cd, 0., 5);
  auto morisonForce1 = make_morison_force(morisonModel1, bodyList[0]);

  auto morisonModel2 = make_morison_model(bodyList[1].get());
  //morisonModel2->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
  morisonModel2->AddElement(Position(-0.866, -0.5, 0.), Position(0.866, 0.5, 0.), 1., 0., cd, 0., 5);
  auto morisonForce2 = make_morison_force(morisonModel2, bodyList[1]);

  auto morisonModel3 = make_morison_model(bodyList[2].get());
  //morisonModel3->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
  morisonModel3->AddElement(Position(-0.866, +0.5, 0.), Position(0.866, -0.5, 0.), 1., 0., cd, 0., 5);
  auto morisonForce3 = make_morison_force(morisonModel3, bodyList[2]);

  auto morisonModel4 = make_morison_model(bodyList[3].get());
  //morisonModel4->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
  morisonModel4->AddElement(Position(0., -1, 0.), Position(0., 1., 0.), 1., 0., cd, 0., 5);
  auto morisonForce4 = make_morison_force(morisonModel4, bodyList[3]);

  auto morisonModel5 = make_morison_model(bodyList[4].get());
  //morisonModel5->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
  morisonModel5->AddElement(Position(-0.866, -0.5, 0.), Position(0.866, 0.5, 0.), 1., 0., cd, 0., 5);
  auto morisonForce5 = make_morison_force(morisonModel5, bodyList[4]);

  auto morisonModel6 = make_morison_model(bodyList[5].get());
  //morisonModel6->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
  morisonModel6->AddElement(Position(-0.866, +0.5, 0.), Position(0.866, -0.5, 0.), 1., 0., cd, 0., 5);
  auto morisonForce6 = make_morison_force(morisonModel6, bodyList[5]);

}

void SetUpMooringLines(FrOffshoreSystem *system, std::vector<std::shared_ptr<FrBody>> bodyList) {

  bool elastic = false;
  double uLength = 12.5;

  auto cableProp = make_cable_properties();
  cableProp->SetSectionArea(0.00196);
  cableProp->SetEA(2.95E5);
  cableProp->SetLinearDensity(9.03);

  auto worldNodeNW = system->GetWorldBody()->NewNode();
  worldNodeNW->SetPositionInWorld(Position(-10, 5.76, -7.5), NWU);
  auto bodyNodeNW = bodyList[0]->NewNode();
  bodyNodeNW->SetPositionInWorld(Position(-2.498, 1.444, 0.), NWU);

  auto worldNodeN = system->GetWorldBody()->NewNode();
  worldNodeN->SetPositionInWorld(Position(0., 11.548, -7.5), NWU);
  auto bodyNodeN = bodyList[1]->NewNode();
  bodyNodeN->SetPositionInWorld(Position(0., 2.887, 0.), NWU);

  auto worldNodeNE = system->GetWorldBody()->NewNode();
  worldNodeNE->SetPositionInWorld(Position(10., 5.76, -7.5), NWU);
  auto bodyNodeNE = bodyList[2]->NewNode();
  bodyNodeNE->SetPositionInWorld(Position(2.498, 1.444, 0.), NWU);

  auto worldNodeSE = system->GetWorldBody()->NewNode();
  worldNodeSE->SetPositionInWorld(Position(10., -5.76, -7.5), NWU);
  auto bodyNodeSE = bodyList[3]->NewNode();
  bodyNodeSE->SetPositionInWorld(Position(2.498, -1.444, 0.), NWU);

  auto worldNodeS = system->GetWorldBody()->NewNode();
  worldNodeS->SetPositionInWorld(Position(0., -11.548, -7.5), NWU);
  auto bodyNodeS = bodyList[4]->NewNode();
  bodyNodeS->SetPositionInWorld(Position(0., -2.887, 0.), NWU);

  auto worldNodeSW = system->GetWorldBody()->NewNode();
  worldNodeSW->SetPositionInWorld(Position(-10., -5.76, -7.5), NWU);
  auto bodyNodeSW = bodyList[5]->NewNode();
  bodyNodeSW->SetPositionInWorld(Position(-2.498, -1.444, 0.), NWU);

  auto mooringLineNW = make_catenary_line(worldNodeNW, bodyNodeNW, system, cableProp, elastic, uLength,
                                          FLUID_TYPE::WATER);
  auto mooringLineN = make_catenary_line(worldNodeN, bodyNodeN, system, cableProp, elastic, uLength, FLUID_TYPE::WATER);
  auto mooringLineNE = make_catenary_line(worldNodeNE, bodyNodeNE, system, cableProp, elastic, uLength,
                                          FLUID_TYPE::WATER);
  auto mooringLineSE = make_catenary_line(worldNodeSE, bodyNodeSE, system, cableProp, elastic, uLength,
                                          FLUID_TYPE::WATER);
  auto mooringLineS = make_catenary_line(worldNodeS, bodyNodeS, system, cableProp, elastic, uLength, FLUID_TYPE::WATER);
  auto mooringLineEW = make_catenary_line(worldNodeSW, bodyNodeSW, system, cableProp, elastic, uLength,
                                          FLUID_TYPE::WATER);

}

void SetUpCurrentForces(FrOffshoreSystem *system, std::vector<std::shared_ptr<FrBody>> bodyList) {

  auto currentForce1 = make_quadratic_damping_force(bodyList[0], WATER, true);
  currentForce1->SetProjectedSections(2., MU_PI, 2.);
  currentForce1->SetDampingCoefficients(1., 1., 1.);

  auto currentForce2 = make_quadratic_damping_force(bodyList[1], WATER, true);
  currentForce2->SetProjectedSections(1.5, 1.5, 2.);
  currentForce2->SetDampingCoefficients(1., 1., 1.);

  auto currentForce3 = make_quadratic_damping_force(bodyList[2], WATER, true);
  currentForce3->SetProjectedSections(1.5, 1.5, 2.);
  currentForce3->SetDampingCoefficients(1., 1., 1.);

  auto currentForce4 = make_quadratic_damping_force(bodyList[3], WATER, true);
  currentForce4->SetProjectedSections(2., MU_PI, 2.);
  currentForce4->SetDampingCoefficients(1., 1., 1.);

  auto currentForce5 = make_quadratic_damping_force(bodyList[4], WATER, true);
  currentForce5->SetProjectedSections(1.5, 1.5, 2.);
  currentForce5->SetDampingCoefficients(1., 1., 1.);

  auto currentForce6 = make_quadratic_damping_force(bodyList[5], WATER, true);
  currentForce6->SetProjectedSections(1.5, 1.5, 2.);
  currentForce6->SetDampingCoefficients(1., 1., 1.);

}

int main(int argc, char *argv[]) {

  // System

  FrOffshoreSystem system;

//    system.GetPathManager()->SetLogOutputPath("../results");
//    system.GetPathManager()->SetLogFrameConvention(NWU);
  system.GetPathManager()->SetResourcesPath(std::string(RESOURCES_PATH));
  system.SetName("HexagonalArticulatedBuoy");

  // Environment
  SetUpEnvironment(&system);

  // Bodies
  std::vector<std::shared_ptr<FrBody>> bodyList;

  auto cyl1 = CreateBody(&system, Position(-2.498, 0., 0.), FrRotation());
  cyl1->SetName("cyl1");
  bodyList.push_back(cyl1);

  auto cyl2 = CreateBody(&system, Position(-1.25, 2.165, 0.), FrRotation(Direction(0., 0., 1.), -M_PI / 3., NWU));
  cyl2->SetName("cyl2");
  bodyList.push_back(cyl2);

  auto cyl3 = CreateBody(&system, Position(1.25, 2.165, 0.), FrRotation(Direction(0., 0., 1.), M_PI / 3., NWU));
  cyl3->SetName("cyl3");
  bodyList.push_back(cyl3);

  auto cyl4 = CreateBody(&system, Position(2.498, 0., 0.), FrRotation());
  cyl4->SetName("cyl4");
  bodyList.push_back(cyl4);

  auto cyl5 = CreateBody(&system, Position(1.25, -2.165, 0.), FrRotation(Direction(0., 0., 1.), -M_PI / 3., NWU));
  cyl5->SetName("cyl5");
  bodyList.push_back(cyl5);

  auto cyl6 = CreateBody(&system, Position(-1.25, -2.165, 0.), FrRotation(Direction(0., 0., 1.), M_PI / 3., NWU));
  cyl6->SetName("cyl6");
  bodyList.push_back(cyl6);

  // Links
  LinkBodies(&system, bodyList);

  // Hydrodynamics

  auto hdb = SetUpHydrodynamicModel(&system, bodyList);

  // Excitation

  auto excitationForce1 = make_linear_excitation_force(hdb, cyl1);
  auto excitationForce2 = make_linear_excitation_force(hdb, cyl2);
  auto excitationForce3 = make_linear_excitation_force(hdb, cyl3);
  auto excitationForce4 = make_linear_excitation_force(hdb, cyl4);
  auto excitationForce5 = make_linear_excitation_force(hdb, cyl5);
  auto excitationForce6 = make_linear_excitation_force(hdb, cyl6);

  // Hydrostatic

  auto cyl1Mesh = make_hydro_mesh(cyl1, system.GetDataPath("cylinder_base.obj"),
                                  FrFrame(Position(0., 0., 0.), FrRotation(), NWU),
                                  FrHydroMesh::ClippingSupport::PLANESURFACE);

  auto cyl2Mesh = make_hydro_mesh(cyl2, system.GetDataPath("cylinder_base.obj"),
                                  FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), -M_PI / 3., NWU),
                                          NWU),
                                  FrHydroMesh::ClippingSupport::PLANESURFACE);

  auto cyl3Mesh = make_hydro_mesh(cyl3, system.GetDataPath("cylinder_base.obj"),
                                  FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), M_PI / 3., NWU), NWU),
                                  FrHydroMesh::ClippingSupport::PLANESURFACE);

  auto cyl4Mesh = make_hydro_mesh(cyl4, system.GetDataPath("cylinder_base.obj"),
                                  FrFrame(Position(0., 0., 0.), FrRotation(), NWU),
                                  FrHydroMesh::ClippingSupport::PLANESURFACE);

  auto cyl5Mesh = make_hydro_mesh(cyl5, system.GetDataPath("cylinder_base.obj"),
                                  FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), -M_PI / 3., NWU),
                                          NWU),
                                  FrHydroMesh::ClippingSupport::PLANESURFACE);

  auto cyl6Mesh = make_hydro_mesh(cyl6, system.GetDataPath("cylinder_base.obj"),
                                  FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), M_PI / 3., NWU), NWU),
                                  FrHydroMesh::ClippingSupport::PLANESURFACE);

  auto forceHst1 = make_nonlinear_hydrostatic_force(cyl1, cyl1Mesh);
  auto forceHst2 = make_nonlinear_hydrostatic_force(cyl2, cyl2Mesh);
  auto forceHst3 = make_nonlinear_hydrostatic_force(cyl3, cyl3Mesh);
  auto forceHst4 = make_nonlinear_hydrostatic_force(cyl4, cyl4Mesh);
  auto forceHst5 = make_nonlinear_hydrostatic_force(cyl5, cyl5Mesh);
  auto forceHst6 = make_nonlinear_hydrostatic_force(cyl6, cyl6Mesh);

  // Morison

  SetUpMorisonModel(&system, bodyList);

  // Mooring Line

  SetUpMooringLines(&system, bodyList);

  // Current force as quadratic damping

  SetUpCurrentForces(&system, bodyList);

  // Simulation

  auto dt = 0.01;

  system.SetTimeStep(dt);
  system.Initialize();

  bool is_irrlicht = true;

  if (is_irrlicht) {
    system.RunInViewer(50., 10., false, 5);
  } else {
    auto time = 0.;
    while (time < 50.) {
      time += dt;
      system.AdvanceTo(time);
      std::cout << "Time : " << time << "s " << std::endl;
    }
  }
  std::cout << " ============================================== End =============================== " << std::endl;
  return 0;
}
