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
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/hydrodynamics/morison/FrMorisonModel.h"

using namespace frydom;
using namespace chrono;

void PlotResults(const std::vector<double> &vtime, const std::vector<double> &vx,
                 const std::vector<double> &vy, const std::vector<double> &vz,
                 const std::string label = "velocity") {

  matplotlibcpp::named_plot(label + ".x", vtime, vx);
  matplotlibcpp::named_plot(label + ".y", vtime, vy);
  matplotlibcpp::named_plot(label + ".z", vtime, vz);
  matplotlibcpp::xlabel("time (s)");
  matplotlibcpp::ylabel(label + " (SI)");
  matplotlibcpp::grid;
  matplotlibcpp::legend();
  matplotlibcpp::show();

}


int main(int argc, char *argv[]) {

  // ---------------------------------------------------------
  // System
  // ---------------------------------------------------------

  FrOffshoreSystem system;

  // ---------------------------------------------------------
  // Environment
  // ---------------------------------------------------------

  //system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_IRREGULAR);
  //auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
  //waveField->SetMeanWaveDirection(0., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
  //waveField->SetWavePulsations(0.5, 2., 80, RADS);
  //waveField->GetWaveSpectrum()->SetHs(3.);
  //waveField->GetWaveSpectrum()->SetTp(10.);

  system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_REGULAR);
  auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
  waveField->SetRegularWaveHeight(3.);
  waveField->SetRegularWavePeriod(10.);
  waveField->SetMeanWaveDirection(0., DEG);

  waveField->GetSteadyElevation(0, 0);

  waveField->GetWaveRamp()->Deactivate();

  system.GetEnvironment()->SetCurrent(FrCurrent::UNIFORM);
  system.GetEnvironment()->GetCurrent()->Set(NORTH, 0., KNOT, NED, COMEFROM);

  // ----------------------------------------------------------
  // Morison structure
  // ----------------------------------------------------------

  auto structure = std::make_shared<FrHydroBody>();
  structure->SetPos(chrono::ChVector<double>(0, 0, -1));

  structure->SetMass(0.);

  auto morison = std::make_shared<FrCompositeElement>();

  morison->SetDefaultDrag(0.01);
  morison->SetDefaultDiameter(0.1);
  morison->SetDefaultFriction(0.);
  morison->SetDefaultMass(0.01);

  morison->AddElement(ChVector<>(0., -2, 1), ChVector<>(0., +2., 1.));
  morison->AddElement(ChVector<>(0., -2, 0), ChVector<>(0., +2., 0.));
  morison->AddElement(ChVector<>(0., -2, -1), ChVector<>(0., +2., -1.));
  morison->AddElement(ChVector<>(0., -2, -1), ChVector<>(0., -2., 1.));
  morison->AddElement(ChVector<>(0., 0, -1), ChVector<>(0., 0., 1.));
  morison->AddElement(ChVector<>(0., +2, -1), ChVector<>(0., +2., 1.));

  morison->ActivateGlobalForce();

  morison->SetBody(structure.get());

  system.AddBody(structure);

  structure->SetBodyFixed(true);

  //structure->Set3DOF_ON(chrono::ChVector<>(1, 0, 0));
  //structure->Set3DOF_ON(chrono::ChVector<>(0, 0, 1));

  // ----------------------------------------------------------
  // Simulation
  // ----------------------------------------------------------

  double time = 0.;
  double dt = 0.01;

  std::vector<double> vx, vy, vz, vtime;
  std::vector<double> fx, fy, fz;
  std::vector<double> x, y, z;

  ChVector<double> velocity;
  ChVector<double> force;
  ChVector<double> position;
  std::vector<std::shared_ptr<ChForce>> force_list;

  system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
  system.SetStep(dt);

  system.Initialize();

  std::cout << "Number of force : " << structure->GetForceList().size() << std::endl;

  while (time < 150.) {

    time += dt;

    system.DoStepDynamics(dt);
    //waveField->Update(time);

    structure->UpdateForces(time);

    velocity = waveField->GetVelocity(0, 0, -10);

    vtime.push_back(time);
    vx.push_back(velocity.x());
    vy.push_back(velocity.y());
    vz.push_back(velocity.z());

    force_list = structure->GetForceList();
    force = force_list[0]->GetForce();
    fx.push_back(force.x());
    fy.push_back(force.y());
    fz.push_back(force.z());

    position = structure->GetPos();
    x.push_back(position.x());
    y.push_back(position.y());
    z.push_back(position.z());

  }

  PlotResults(vtime, vx, vy, vz);

  PlotResults(vtime, fx, fy, fz, "force");

  PlotResults(vtime, x, y, z, "position");

  return 0;
}
