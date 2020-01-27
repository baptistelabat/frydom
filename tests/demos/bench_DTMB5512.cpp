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

FrLinearActuator *make_carriage(FrOffshoreSystem *system, const std::shared_ptr<FrNode> &shipNode,
                                bool is_captive) {

  FRAME_CONVENTION fc = NWU;

  auto mass = shipNode->GetBody()->GetInertiaTensor().GetMass();

  double tankLength = 140;
  double tankWidth = 5;
  double tankDepth = 3.048;

  // --------------------------------------------------
  // Seabed and Free-surface grid definitions
  // --------------------------------------------------
  auto Seabed = system->GetEnvironment()->GetOcean()->GetSeabed();
  Seabed->GetSeabedGridAsset()->SetGrid(
      -0.05 * tankLength, 0.95 * tankLength, 0.01 * tankLength,
      -0.5 * tankWidth, 0.5 * tankWidth, 0.01 * tankWidth);
  Seabed->SetBathymetry(-tankDepth, fc);

  auto FreeSurface = system->GetEnvironment()->GetOcean()->GetFreeSurface();
  FreeSurface->GetFreeSurfaceGridAsset()->SetGrid(
      -0.05 * tankLength, 0.95 * tankLength, 0.001 * tankLength,
      -0.5 * tankWidth, 0.5 * tankWidth, 0.01 * tankWidth);
  FreeSurface->GetFreeSurfaceGridAsset()->UpdateAssetON();
  FreeSurface->GetFreeSurfaceGridAsset()->SetUpdateStep(10);

  // --------------------------------------------------
  // Wall definition
  // --------------------------------------------------

  auto tankWall = system->NewBody("Wall");
  tankWall->SetFixedInWorld(true);
  makeItBox(tankWall, tankLength, 0.1 * tankWidth, 1.25 * tankDepth, mass);

  Position tankWallPosition = shipNode->GetPositionInWorld(fc);
  tankWallPosition.GetZ() = 0.;
  tankWallPosition -= Position(-0.45 * tankLength, 0.5 * tankWidth, 0.375 * tankDepth);
  tankWall->SetPosition(tankWallPosition, fc);

  auto wallNode = tankWall->NewNode("wallNode");
  wallNode->SetPositionInBody(Position(-0.45 * tankLength, 0., 0.75 * tankDepth), fc);
  wallNode->RotateAroundYInBody(-90 * DEG2RAD, fc);

  // --------------------------------------------------
  // Carriage definition
  // --------------------------------------------------

  auto carriage = system->NewBody("Carriage");
  carriage->SetPositionOfBodyPoint(Position(0., -0.5 * tankWidth, 0.), wallNode->GetPositionInWorld(fc), fc);

  double xSize = 0.1 * tankWidth;
  double ySize = 1.1 * tankWidth;
  double zSize = 0.1 * tankWidth;

  // Properties of the box
  double xSize2 = xSize * xSize;
  double ySize2 = ySize * ySize;
  double zSize2 = zSize * zSize;

  // inertia
  double Ixx = (1. / 12.) * mass * (ySize2 + zSize2);
  double Iyy = (1. / 12.) * mass * (xSize2 + zSize2);
  double Izz = (1. / 12.) * mass * (xSize2 + ySize2);

  carriage->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., Position(), NWU));

  auto carriageAsset = std::make_shared<FrTriangleMeshConnected>();
  auto carriageMesh = FrFileSystem::join({system->config_file().GetDataFolder(), "ce/bench/DTMB5512/carriage.obj"});
  carriageAsset->LoadWavefrontMesh(carriageMesh);
  carriageAsset->Scale(tankWidth);
  carriage->AddMeshAsset(carriageAsset);

  auto carriageToWallNode = carriage->NewNode("carriageToWallNode");
  carriageToWallNode->SetPositionInBody(Position(0., -0.5 * tankWidth, 0.), fc);
  carriageToWallNode->RotateAroundYInBody(-90 * DEG2RAD, fc);

  auto carriageToShipNode = carriage->NewNode("carriageToShipNode");
  carriageToShipNode->SetPositionInWorld(shipNode->GetPositionInWorld(fc), fc);
  carriageToShipNode->RotateAroundYInBody(90 * DEG2RAD, fc);
  carriageToShipNode->RotateAroundXInBody(90 * DEG2RAD, fc);

  // --------------------------------------------------
  // Link definitions
  // --------------------------------------------------

  if (is_captive)
    auto linkToShip = make_fixed_link("carriage-ship_fixed", system, carriageToShipNode, shipNode);
  else
    auto linkToShip = make_prismatic_revolute_link("carriage-ship_prismatic-revolute", system, carriageToShipNode,
                                                   shipNode);

  auto rail = make_prismatic_link("wall-carriage_prismatic", system, wallNode, carriageToWallNode);

  return rail->Motorize("wall-carriage_actuator", VELOCITY);
}

// ----------------------------------------------------------
// Steady Pitch Torque
// ----------------------------------------------------------

class SteadyPitchTorque : public FrForce {

  void Compute(double time) override {

    auto speed = GetBody()->GetLinearVelocityInWorld(NWU).GetVx();
    auto torque = 4.332 * std::pow(speed, 6)
                  - 9.1135 * std::pow(speed, 5)
                  - 9.7756 * std::pow(speed, 4)
                  + 34.232 * std::pow(speed, 3)
                  - 22.7359 * std::pow(speed, 2);

    SetTorqueInBodyAtCOG(Torque(0., -torque, 0.), NWU);

  }

 public:
  SteadyPitchTorque(const std::string &name, FrBody *body) : FrForce(name, "SteadyPitchTorque", body) {}
};

// ----------------------------------------------------------
// Steady Heave Force
// ----------------------------------------------------------

class SteadyHeaveForce : public FrForce {

  void Compute(double time) override {

    auto speed = GetBody()->GetLinearVelocityInWorld(NWU).GetVx();

    auto force = -12.32426 * std::pow(speed, 3)
                 - 2.8696 * std::pow(speed, 2);

    SetForceInWorldAtCOG(Force(0., 0., force), NWU);

  }

 public:
  SteadyHeaveForce(const std::string &name, FrBody *body) : FrForce(name, "SteadyHeaveForce", body) {}
};

// -----------------------------------------------------------
// ITTC57 : residual coefficient
// -----------------------------------------------------------

double ResidualITTC(double speed) {

  if (std::abs(speed - 1.04) < 1E-2) {
    return 5.3696e-4;
  } else if (std::abs(speed - 1.532) < 1E-2) {
    return 9.0677e-4;
  } else if (std::abs(speed - 1.86) < 1E-2) {
    return 1.6812e-3;
  } else if (std::abs(speed - 2.243) < 1E-2) {
    return 4.02529e-3;
  } else {
    std::cout << "warning : no residual coefficient for this speed value" << std::endl;
    std::cout << "        : residual set to 0. " << std::endl;
  }
  return 0.;
};


// ------------------------------------------------------------
// Benchmark : main
// ------------------------------------------------------------

int main(int argc, char *argv[]) {

  std::cout << " ======================================================= \n"
               " Benchmark test : DTMB 5512 captive motion \n"
               " =======================================================" << std::endl;

  // -- Input

  double speed = atof(argv[1]);   // Ship forward speed
  double ak = 0.5 * atof(argv[2]);  // Wave amplitude (m)
  double Tk = atof(argv[3]);      // Wave period (s)
  char *name = argv[4];     // Output director prefix name

  bool captive_test = true;      // fixed heave and pitch motions

  // -- System

  FrOffshoreSystem system(name);

  // -- Ocean
  auto ocean = system.GetEnvironment()->GetOcean();
  ocean->SetDensity(1000.);

  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWaveHeight(ak);
  waveField->SetWavePeriod(Tk);
  waveField->SetDirection(180., DEG, NWU, GOTO);

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(5., 0., 20., 1.);

  // -- Body

  auto body = system.NewBody("DTMB");
  Position COGPosition(0., 0., 0.03); // 0.03
  body->SetPosition(Position(0., 0., 0.), NWU);
  auto DTMB_asset = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/DTMB5512/DTMB5512_close.obj"});
  body->AddMeshAsset(DTMB_asset);
  body->SetColor(Yellow);

  // -- Inertia

  double mass = 86.0;

  double Ixx = 1.98;
  double Iyy = 53.88;
  double Izz = 49.99;

  FrInertiaTensor InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGPosition, NWU);
  body->SetInertiaTensor(InertiaTensor);

  // -- Hydrodynamics

  auto DTMB_hdb = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/DTMB5512/DTMB5512.hdb5"});
  auto hdb = make_hydrodynamic_database(DTMB_hdb);

  auto eqFrame = make_equilibrium_frame("EqFrame", &system, body, {0., 0., 0.03}, 0., NWU);
  eqFrame->SetVelocityInWorld({speed, 0., 0.}, NWU);

  hdb->Map(0, body.get(), eqFrame);

  // -- Hydrostatic

  auto forceHst = make_linear_hydrostatic_force("linear_hydrostatic", body, hdb);

  // -- Radiation

  auto radiationModel = make_radiation_convolution_model("radiation_convolution", &system, hdb);
  radiationModel->SetImpulseResponseSize(body.get(), 50., 0.008);

  // -- Excitation

  auto excitationForce = make_linear_excitation_force("linear_excitation", body, hdb);

  // -- Wave Drift force

  auto waveDriftForce = make_wave_drift_force("wave_drift", body, hdb);

  // -- ITTC57

  auto lpp = 3.048;
  auto wettedSurfaceArea = 1.371;

  auto ct = ResidualITTC(speed);
  auto forceResistance = make_ITTC_resistance_force("ITTC_resistance", body, lpp, wettedSurfaceArea, ct, 0.03);

  // -- Steady force

  auto forcePitch = std::make_shared<SteadyPitchTorque>("forcePitch", body.get());
  body->AddExternalForce(forcePitch);

  auto forceHeave = std::make_shared<SteadyHeaveForce>("forceHeave", body.get());
  body->AddExternalForce(forceHeave);

  // -- Carriage and fixation point

  auto shipNode = body->NewNode("shipNode");
  shipNode->SetPositionInBody(body->GetCOG(NWU), NWU);
  shipNode->RotateAroundYInBody(90 * DEG2RAD, NWU);
  shipNode->RotateAroundXInBody(90 * DEG2RAD, NWU);

  auto carriage = make_carriage(&system, shipNode, captive_test);
  carriage->SetMotorFunction(FrConstantFunction(speed));

  auto dt = 0.008;

  system.SetTimeStep(dt);
  system.Initialize();
  system.DoAssembly();

  bool is_irrlicht = true;

  if (is_irrlicht) {
    system.RunInViewer(50., 10., false);
  } else {
    double time = 0.;
    while (time < 50.) {
      time += dt;
      system.AdvanceTo(time);
      std::cout << "time : " << time << std::endl;
    }
  }
  std::cout << "=============================== End ========================" << std::endl;
}
