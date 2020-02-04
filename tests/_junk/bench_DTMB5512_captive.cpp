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

#include <iostream>
#include <fstream>
#include <vector>

#include "frydom/frydom.h"
#include "matplotlibcpp.h"

using namespace frydom;
using namespace chrono;

void PlotData(std::vector<double> &vtime, std::vector<double> &ydata, std::string ylabel) {

  matplotlibcpp::plot(vtime, ydata);
  matplotlibcpp::xlabel("time (s)");
  matplotlibcpp::ylabel(ylabel);

}

void PlotResult(std::vector<double> &vtime, std::vector<ChVector<double>> &vposition,
                std::vector<ChVector<double>> &vrotation, int idata) {

  std::vector<double> x, y, z;
  std::vector<double> rx, ry, rz;

  for (auto position: vposition) {
    x.push_back(position.x());
    y.push_back(position.y());
    z.push_back(position.z());
  }

  for (auto rotation: vrotation) {
    rx.push_back(rotation.x());
    ry.push_back(rotation.y());
    rz.push_back(rotation.z());
  }

  std::string ylabel1, ylabel2;

  switch (idata) {
    case 0:
      ylabel1 = "position (m)";
      ylabel2 = "rotation (rad)";
      break;
    case 1:
      ylabel1 = "force (N)";
      ylabel2 = "torque (N.m)";
      break;
  }
  matplotlibcpp::subplot(2, 3, 1);
  PlotData(vtime, x, ylabel1);
  matplotlibcpp::title("surge");

  matplotlibcpp::subplot(2, 3, 2);
  PlotData(vtime, y, ylabel1);
  matplotlibcpp::title("sway");

  matplotlibcpp::subplot(2, 3, 3);
  PlotData(vtime, z, ylabel1);
  matplotlibcpp::title("heave");

  matplotlibcpp::subplot(2, 3, 4);
  PlotData(vtime, rx, ylabel2);
  matplotlibcpp::title("roll");

  matplotlibcpp::subplot(2, 3, 5);
  PlotData(vtime, ry, ylabel2);
  matplotlibcpp::title("pitch");

  matplotlibcpp::subplot(2, 3, 6);
  PlotData(vtime, rz, ylabel2);
  matplotlibcpp::title("yaw");

  matplotlibcpp::show();
}


// ----------------------------------------------------------
// ship model : DTMB5512 (IIHR - Yoon 2009)
// ----------------------------------------------------------

std::shared_ptr<FrShip> DTMB5512(FrOffshoreSystem *system) {

  auto ship_pos = ChVector<>(0., 0., 0.);

  auto ship = std::make_shared<FrShip>();
  system->AddBody(ship);

  // Geometry properties
  ship->SetAbsPos(ChVector<>(0., 0., 0.));
  ship->SetName("DTMB5512");
  ship->SetHydroMesh("DTMB5512.obj", true);
  ship->SetLpp(3.048);
  ship->SetMass(86.0);
  ship->SetWettedSurface(1.371);
  ship->SetCOG(ChVector<>(0., 0., +0.03));
  //ship->SetInertiaXX(ChVector<>(1.98, 53.88, 49.99));
  ship->SetInertiaXX(ChVector<>(1.98, 53.88, 49.99));
  ship->SetEquilibriumFrame(MeanMotion, 60.);

  // Hydrostatics
  auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
  auto hstStiffness = hstForce->GetStiffnessMatrix();

  hstStiffness->SetDiagonal(9.68e3, 3.46e1, 5.42e3);
  //hstStiffness->SetDiagonal(9.976e3, 2.7973e1, 5.42e3);
  hstStiffness->SetNonDiagonal(0., +1.25e3, 0.);
  // data from Boris :
  //hstStiffness->SetDiagonal(9.279095e3, 2.881106e1, 5.006731e3);
  //hstStiffness->SetNonDiagonal(0., 1.013974e3, 0.);

  ship->AddForce(hstForce);

  // Hydrodynamics
  system->SetHydroDB("DTMB5512_VCG003_Xforward_opposite_dir.hdb5");
  auto hydroMapIndex = system->GetHydroMapNb() - 1;
  system->GetHydroMapper(hydroMapIndex)->Map(ship, 0);

  // Radiation model
  auto radModel = std::make_shared<FrRadiationConvolutionModel>(system->GetHydroDB(hydroMapIndex), system);
  radModel->SetHydroMapIndex(hydroMapIndex); // TODO : patch hydro map multibody
  radModel->AddRadiationForceToHydroBody(ship);
  radModel->SetSpeedDependent();

  // Wave Probe
  auto waveField = system->GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
  //auto waveProbe = waveField->NewWaveProbe(ship_pos.x(), ship_pos.y());
  auto waveProbe = std::make_shared<FrLinearWaveProbe>();
  waveProbe->AttachedNode(ship->GetEquilibriumFrame());
  waveField->AddWaveProbe(waveProbe);
  waveProbe->Initialize();

  // Wave Excitation force
  auto excForce = std::make_shared<FrLinearExcitationForce>();
  ship->AddForce(excForce);
  excForce->SetWaveProbe(waveProbe);
  excForce->SetHydroMapIndex(hydroMapIndex);
  excForce->SetSteady(true);

  // Standard current viscous drag force model
  //auto drag_force = std::make_shared<FrCurrentStandardForce>(ship);
  //drag_force->SetDraft(0.132);
  //drag_force->SetMaxBreadth(0.42);
  //ship->AddForce(drag_force);

  // Wave drift force
  auto waveDrift = std::make_shared<FrWaveDriftForce>("DTMB5512_WaveDriftCoeff.h5");
  ship->AddForce(waveDrift);
  waveDrift->SetBody(ship);
  waveDrift->SetWaveProbe(waveProbe);

  // Friction force from ITTC57
  auto forceITTC57 = std::make_shared<FrITTC57>();
  forceITTC57->SetCharacteristicLength(ship->GetLpp());
  forceITTC57->SetHullWettedSurface(ship->GetWettedSurface());
  forceITTC57->SetHullFrontalProjectedArea(ship->GetTransverseUnderWaterArea());
  ship->AddForce(forceITTC57);

  // Man damping
  //auto forceMan = std::make_shared<FrTaylorManDamping>();
  //ship->AddForce(forceMan);
  //forceMan->SetX("Xuuu",-5.59134);
  //forceMan->SetX("Xuuuu", 2.34285);
  //forceMan->SetX("Xuu", 2.720665);

  // Steady force
  auto forcePitch = std::make_shared<FrSteadyPitchTorque>();
  ship->AddForce(forcePitch);

  auto forceHeave = std::make_shared<FrSteadyHeaveForce>();
  ship->AddForce(forceHeave);

  // Logging

  ship->Log().SetNameAndDescription("ShipLog", "Message log of the ship");
  ship->SetLogDefault();

  for (auto force: ship->GetForceList()) {
    auto dforce = dynamic_cast<FrForce *>(force.get());
    ship->Log().AddField<hermes::Message>("force", "-", "external force", dforce->GetLog());
  }

  return ship;
}

void WriteCSV(std::string filename, const std::vector<double> vtime,
              const std::vector<double> wave0, std::vector<double> wave1,
              const std::vector<ChVector<>> vposition, const std::vector<ChVector<>> vrotation,
              const std::vector<ChVector<>> vforce, const std::vector<ChVector<>> vtorque) {

  std::ofstream fid;
  fid.open(filename);

  fid << "t;eta0;eta1;X;Y;Z;RX;RY;RZ;FX;FY;FZ;MX;MY;MZ" << std::endl;

  for (unsigned int i = 0; i < vtime.size(); i++) {
    fid << vtime[i] << ";";
    fid << wave0[i] << ";" << wave1[i] << ";";
    fid << vposition[i].x() << ";" << vposition[i].y() << ";" << vposition[i].z() << ";";
    fid << vrotation[i].x() << ";" << vrotation[i].y() << ";" << vrotation[i].z() << ";";
    fid << vforce[i].x() << ";" << vforce[i].y() << ";" << vforce[i].z() << ";";
    fid << vtorque[i].x() << ";" << vtorque[i].y() << ";" << vtorque[i].z() << std::endl;
  }

  fid.close();

}

void PlotFFT(std::vector<double> vtime, std::vector<double> vfunc,
             double tmin, double tmax) {


  std::vector<double> vx, vy;

  for (unsigned int i = 0; i < vtime.size(); i++) {
    if (vtime[i] > tmin && vtime[i] < tmax) {
      vx.push_back(vtime[i]);
      vy.push_back(vfunc[i]);
    }
  }

  double fs = 1. / (vtime[1] - vtime[0]);

  FFT<double> fft;
  fft.ScalingOFF();
  fft.HalfSpectrumON();

  std::vector<std::complex<double>> freqVect;
  std::vector<double> frequencies;
  fft.fft(freqVect, frequencies, vy, fs, HZ);

  matplotlibcpp::subplot(2, 1, 1);
  matplotlibcpp::plot(frequencies, Amplitude(freqVect));
  matplotlibcpp::grid(true);
  matplotlibcpp::subplot(2, 1, 2);
  matplotlibcpp::plot(frequencies, Phase(freqVect, DEG));
  matplotlibcpp::grid(true);
  matplotlibcpp::show();
}

int main(int argc, char *argv[]) {

  double speed = atof(argv[1]);
  double ak = atof(argv[2]);
  double Tk = atof(argv[3]);
  std::string name = argv[4];

  // ------------------------------------------------------
  // System
  // ------------------------------------------------------

  FrOffshoreSystem system;

  // ------------------------------------------------------
  // Environment
  // ------------------------------------------------------

  system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_REGULAR);
  auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
  waveField->SetRegularWaveHeight(0.0);
  waveField->SetRegularWavePeriod(Tk);
  waveField->SetMeanWaveDirection(180.);
  waveField->GetSteadyElevation(0, 0);
  //waveField->GetWaveRamp()->Deactivate();
  waveField->GetWaveRamp()->SetDuration(20.);
  waveField->GetWaveRamp()->Initialize();

  system.GetEnvironment()->SetCurrent(FrCurrent::UNIFORM);
  system.GetEnvironment()->GetCurrent()->Set(0., 0., DEG, KNOT, NED, GOTO);

  // ------------------------------------------------------
  // Body
  // ------------------------------------------------------

  auto ship = DTMB5512(&system);

  auto vspeed = ChVector<>(speed, 0., 0.);
  ship->SetPos_dt(vspeed);
  ship->SetSteadyVelocity(vspeed);
  ship->SetDOF(false, true, false, true, false, true);

  auto BodyEqFrame = dynamic_cast<FrBody *>(ship->GetEquilibriumFrame().get());
  BodyEqFrame->Log().SetNameAndDescription(name + "_EquilibriumFrame", "Hydrodynamic equilibrium frame");

  // ------------------------------------------------------
  // Monitoring waveProbe
  // ------------------------------------------------------

  auto dynamic_waveProbe = std::make_shared<FrLinearWaveProbe>();
  dynamic_waveProbe->AttachedNode(ship->GetEquilibriumFrame());
  waveField->AddWaveProbe(dynamic_waveProbe);
  dynamic_waveProbe->Initialize();

  auto fixed_waveProbe = waveField->NewWaveProbe(0, 0);
  fixed_waveProbe->Initialize();


  // ------------------------------------------------------
  // Simulation
  // ------------------------------------------------------

  std::vector<ChVector<double>> vposition;
  std::vector<ChVector<double>> vrotation;
  std::vector<ChVector<double>> vforce;
  std::vector<ChVector<double>> vtorque;
  std::vector<double> waveElevation0, waveElevation1;
  std::vector<ChVector<double>> posProbe1, speedProbe1;
  std::vector<double> vtime;

  double dt = 0.008;

  auto shipPos = ship->GetFrame_REF_to_abs();
  auto eqFramePos = ship->GetEquilibriumFrame()->GetPos();

  system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
  system.SetStep(dt);
  system.Initialize();

  shipPos = ship->GetFrame_REF_to_abs();
  eqFramePos = ship->GetEquilibriumFrame()->GetPos();

  auto is_irrlicht = false;

  if (is_irrlicht) {

    auto app = FrIrrApp(system);
    app.AddTypicalCamera(irr::core::vector3df(0, 0, 10), irr::core::vector3df(0, 0, -1));

    app.Run();

  } else {

    ChVector<double> global_force, local_force;
    ChVector<double> global_torque, local_torque;

    auto time = 0.;

    while (time < 40.) {

      // Do step

      if (std::abs(time - 5.) < 0.5 * dt) {
        waveField->SetRegularWaveHeight(ak);
        waveField->Initialize();
      }

      system.DoStepDynamics(dt);
      time += dt;
      ship->Update();
      ship->GetPos_dt().x() = vspeed.x();
      vtime.push_back(time);

      //system.StepFinalize();

      // Save ship state

      vposition.push_back(ship->GetPosition());
      vrotation.push_back(quat_to_euler(ship->GetRot()));

      global_force = ChVector<double>();
      global_torque = ChVector<double>();

      for (auto &force: ship->GetForceList()) {
        force->GetBodyForceTorque(local_force, local_torque);
        global_force += local_force;
        global_torque += local_torque;
      }
      // Add gravity
      global_force += ship->GetSystem()->Get_G_acc() * ship->GetMass();

      vforce.push_back(global_force);
      vtorque.push_back(global_torque);

      // Monitor wave elevation

      waveElevation0.push_back(fixed_waveProbe->GetElevation(time));
      waveElevation1.push_back(dynamic_waveProbe->GetElevation(time));
      posProbe1.push_back(dynamic_waveProbe->GetNode()->GetPos());
      speedProbe1.push_back(dynamic_waveProbe->GetNode()->GetPos_dt());
    }

    //  ------------------ Adimentionalize --------------

    auto adim_x = 1. / (0.5 * 1025. * speed * speed * ship->GetWettedSurface());
    auto adim_cm = 1. / (0.5 * 1025 * speed * speed * ship->GetWettedSurface() * ship->GetLpp());

    for (auto &force: vforce) {
      force.x() = adim_x * force.x();
      force.z() = adim_x * force.z();
    }

    for (auto &torque: vtorque) {
      torque.y() = adim_cm * torque.y();
    }

    std::vector<double> vtime_T;
    for (auto &time: vtime) {
      vtime_T.push_back(time / Tk);
    }

    // --------------- Plot position of the ship -----------------

//        PlotResult(vtime, vposition, vrotation, 0);

    // ----------------- Plot external force on the ship ---------

//        PlotResult(vtime, vforce, vtorque, 1);

    // ----------------- Wave Elevation Plot ----------------------
/*
        matplotlibcpp::named_plot("fixed", vtime_T, waveElevation0);
        matplotlibcpp::named_plot("dynamic", vtime_T, waveElevation1);
        matplotlibcpp::xlabel("t/T");
        matplotlibcpp::ylabel("wave elevation (m)");
        matplotlibcpp::legend();
        matplotlibcpp::show();
*/
    // -------------- Equilibrium Frame Position -------------------
/*
        std::vector<double> xp, yp, zp;
        for (auto& pos: posProbe1) {
            xp.push_back(pos.x());
            yp.push_back(pos.y());
            zp.push_back(pos.z());
        }

        matplotlibcpp::title("Position of the equilibrium frame");
        matplotlibcpp::subplot(3,1,1);
        matplotlibcpp::plot(vtime, xp);
        matplotlibcpp::xlabel("time (s)");
        matplotlibcpp::ylabel("X (m)");
        matplotlibcpp::subplot(3,1,2);
        matplotlibcpp::plot(vtime, yp);
        matplotlibcpp::xlabel("time (s)");
        matplotlibcpp::ylabel("Y (m)");
        matplotlibcpp::subplot(3, 1, 3);
        matplotlibcpp::plot(vtime, zp);
        matplotlibcpp::xlabel("time (s)");
        matplotlibcpp::ylabel("Z (m)");
        matplotlibcpp::show();
*/
    // ------------- Write CVS output -----------------------

    //WriteCSV( argv[4], vtime, waveElevation0, waveElevation1, vposition, vrotation, vforce, vtorque);

    // ---------------- User test ---------------------------
/*
        std::vector<double> Fz;
        for (auto& force:vforce) {
            Fz.push_back(force.z());
        }

        std::vector<double> posZ;
        for (auto& position:vposition) {
            posZ.push_back(position.z());
        }

        matplotlibcpp::subplot(3,1,1);
        matplotlibcpp::named_plot("dynamic", vtime, waveElevation1);
        matplotlibcpp::xlabel("t (s)");
        matplotlibcpp::ylabel("eta (m)");
        matplotlibcpp::grid(true);

        matplotlibcpp::subplot(3,1,2);
        matplotlibcpp::plot(vtime, posZ);
        matplotlibcpp::xlabel("time (s)");
        matplotlibcpp::ylabel("Z (m)");
        matplotlibcpp::grid(true);

        matplotlibcpp::subplot(3,1,3);
        matplotlibcpp::plot(vtime, Fz);
        matplotlibcpp::xlabel("t (s)");
        matplotlibcpp::ylabel("Fz (N)");
        matplotlibcpp::grid(true);

        matplotlibcpp::show();
*/


  }

// ==========================================================
}

