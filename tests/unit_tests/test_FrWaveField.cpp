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

#include "gtest/gtest.h"

using namespace frydom;


TEST(FrWaveField, regularWaveField) {

  // Frame convention
  FRAME_CONVENTION fc = NWU;
  // Wave direction convention
  DIRECTION_CONVENTION dc = GOTO;

  // create a system
  FrOffshoreSystem system("test_FrWaveField");

  // Airy regular wave parameters
  double waveHeight = 1.;
  double wavePeriod = 2. * M_PI;
  Direction waveDirection = Direction(NORTH(fc));

  auto omega = 2. * M_PI / wavePeriod;
  auto k = omega * omega / system.GetGravityAcceleration();
  auto waveLength = 2. * M_PI / k;

  // Set depth to infinite
  system.GetEnvironment()->GetOcean()->SetInfiniteDepth();

  // Set the waveField to AiryRegular
  auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();
  auto waveField = freeSurface->SetAiryRegularWaveField();
  waveField->SetWaveHeight(waveHeight);
  waveField->SetWavePeriod(wavePeriod);
  waveField->SetDirection(waveDirection, fc, dc);

  system.Initialize();

  // Test getters
  EXPECT_NEAR(waveHeight, waveField->GetWaveHeight(), 1e-8);
  EXPECT_NEAR(wavePeriod, waveField->GetWavePeriod(S), 1e-8);
  EXPECT_NEAR(waveLength, waveField->GetWaveLength(), 1e-8);
  EXPECT_NEAR(0., waveField->GetDirectionAngle(DEG, fc, dc), 1e-8);
  Direction testDirection = Direction(NORTH(fc)) - waveField->GetDirection(fc, dc);
  EXPECT_TRUE(testDirection.isZero());

  // test at T = 0s
  //          test Elevation
  EXPECT_NEAR(0, waveField->GetElevation(0., 0., fc), 1e-8);
  EXPECT_NEAR(0, waveField->GetElevation(waveLength, 0., fc), 1e-8);
  EXPECT_NEAR(0, -waveField->GetElevation(0.5 * waveLength, 0., fc), 1e-8);
  EXPECT_NEAR(0, waveField->GetElevation(0, 1., fc), 1e-8);
  //          test Velocity
  EXPECT_NEAR(0., waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(0., waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(waveHeight * omega, -waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
  EXPECT_NEAR(waveHeight * omega * omega, -waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(0., waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(0., waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);

  // test at T = Tw/4
  system.AdvanceOneStep(0.25 * wavePeriod);
  EXPECT_NEAR(0.25 * wavePeriod, system.GetTime(), 1e-8);
  //          test Elevation
  EXPECT_NEAR(waveHeight, -waveField->GetElevation(0., 0., fc), 1e-8);
  EXPECT_NEAR(waveHeight, -waveField->GetElevation(waveLength, 0., fc), 1e-8);
  EXPECT_NEAR(waveHeight, waveField->GetElevation(0.5 * waveLength, 0., fc), 1e-8);
  EXPECT_NEAR(waveHeight, -waveField->GetElevation(0, 1., fc), 1e-8);
  //          test Velocity
  EXPECT_NEAR(waveHeight * omega, -waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(0., waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(0., waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
  EXPECT_NEAR(0., waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(0., waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(waveHeight * omega * omega, waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);

  // test GOTO
  system.AdvanceOneStep(0.25 * wavePeriod); // to be sure the wave is not symmetric around the origin
  auto testElevation = waveField->GetElevation(-0.1 * waveLength, 0., fc);
  system.AdvanceOneStep(0.1 * wavePeriod);
  EXPECT_NEAR(testElevation, waveField->GetElevation(0., 0., fc), 1e-8);


}

struct BiChromaticWaveInfDepth {
  double m_depth = 1000;
  double m_T1 = 12;
  double m_T2 = 6;
  double m_w1 = 2. * M_PI / m_T1;;
  double m_w2 = 2. * M_PI / m_T2;
  double m_k1 = SolveWaveDispersionRelation(m_depth, m_w1, 9.81);
  double m_k2 = SolveWaveDispersionRelation(m_depth, m_w2, 9.81);
  double m_Amp = sqrt(2. * (m_w2 - m_w1));
  double m_dir = 0;

  BiChromaticWaveInfDepth(double T1, double T2, double dir, double depth) {
    m_T1 = T1;
    m_T2 = T2, m_dir = dir;
    m_depth = depth;
    Init();
  }

  void Init() {
    m_w1 = 2. * M_PI / m_T1;;
    m_w2 = 2. * M_PI / m_T2;
    m_k1 = SolveWaveDispersionRelation(m_depth, m_w1, 9.81);
    m_k2 = SolveWaveDispersionRelation(m_depth, m_w2, 9.81);
    m_Amp = sqrt(2. * (m_w2 - m_w1));
  }

  double GetElevation(double x, double y, double t) {
    double kdir = x * cos(m_dir) + y * sin(m_dir);
    return m_Amp * (sin(m_k1 * kdir - m_w1 * t) + sin(m_k2 * kdir - m_w2 * t));
  }

  double GetPotential(double x, double y, double z, double t) {
    double kdir = x * cos(m_dir) + y * sin(m_dir);
    double g = 9.81;
    return -m_Amp * (g / m_w1 * cos(m_k1 * kdir - m_w1 * t) * exp(m_k1 * z)
                     + g / m_w2 * cos(m_k2 * kdir - m_w2 * t) * exp(m_k2 * z));
  }

  double GetVx(double x, double y, double z, double t) {
    double kdir = x * cos(m_dir) + y * sin(m_dir);
    return m_Amp * (m_w1 * cos(m_dir) * sin(m_k1 * kdir - m_w1 * t) * exp(m_k1 * z)
                    + m_w2 * cos(m_dir) * sin(m_k2 * kdir - m_w2 * t) * exp(m_k2 * z));
  }

  double GetVy(double x, double y, double z, double t) {
    double kdir = x * cos(m_dir) + y * sin(m_dir);
    return m_Amp * (m_w1 * sin(m_dir) * sin(m_k1 * kdir - m_w1 * t) * exp(m_k1 * z)
                    + m_w2 * sin(m_dir) * sin(m_k2 * kdir - m_w2 * t) * exp(m_k2 * z));
  }

  double GetVz(double x, double y, double z, double t) {
    double kdir = x * cos(m_dir) + y * sin(m_dir);
    return -m_Amp * (m_w1 * cos(m_k1 * kdir - m_w1 * t) * exp(m_k1 * z)
                     + m_w2 * cos(m_k2 * kdir - m_w2 * t) * exp(m_k2 * z));
  }

  double GetAx(double x, double y, double z, double t) {
    double kdir = x * cos(m_dir) + y * sin(m_dir);
    return -m_Amp * (m_w1 * m_w1 * cos(m_dir) * cos(m_k1 * kdir - m_w1 * t) * exp(m_k1 * z)
                     + m_w2 * m_w2 * cos(m_dir) * cos(m_k2 * kdir - m_w2 * t) * exp(m_k2 * z));
  }

  double GetAy(double x, double y, double z, double t) {
    double kdir = x * cos(m_dir) + y * sin(m_dir);
    return -m_Amp * (m_w1 * m_w1 * sin(m_dir) * cos(m_k1 * kdir - m_w1 * t) * exp(m_k1 * z)
                     + m_w2 * m_w2 * sin(m_dir) * cos(m_k2 * kdir - m_w2 * t) * exp(m_k2 * z));
  }

  double GetAz(double x, double y, double z, double t) {
    double kdir = x * cos(m_dir) + y * sin(m_dir);
    return -m_Amp * (m_w1 * m_w1 * sin(m_k1 * kdir - m_w1 * t) * exp(m_k1 * z)
                     + m_w2 * m_w2 * sin(m_k2 * kdir - m_w2 * t) * exp(m_k2 * z));
  }


};


TEST(FrWaveField, BiChromaticWaveInfDepth) {
  double g = 9.81;

  // Frame convention
  FRAME_CONVENTION fc = NWU;
  // Wave direction convention
  DIRECTION_CONVENTION dc = GOTO;

  // create a system
  FrOffshoreSystem system("test_BiChromaticWaveInfDepth");

  // Set depth to infinite
  system.GetEnvironment()->GetOcean()->SetInfiniteDepth();
  double depth = -1000;
//    system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(depth, fc);

  // Set the waveField to AiryRegular
  auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();
  auto waveField = freeSurface->SetAiryIrregularWaveField();

//    // Set the JONSWAP wave spectrum
//    double Hs = 3;
//    double Tp = 9;
//    auto Jonswap = waveField->SetJonswapWaveSpectrum(Hs, Tp);

  auto spectre = waveField->SetTestWaveSpectrum();
  double T1 = 12, T2 = 6;
  auto w1 = 2. * M_PI / T1;
  auto w2 = 2. * M_PI / T2;
//    std:: cout<<"w1 = "<<w1<<", w2 = "<<w2<<std::endl;
  auto k1 = SolveWaveDispersionRelation(depth, w1, g);
  auto k2 = SolveWaveDispersionRelation(depth, w2, g);
  double Amp = sqrt(2. * (w2 - w1));
//    std::cout<<"Amp = "<<Amp<<std::endl;

  waveField->SetWaveFrequencies(w1, w2, 2);

  // Set wave direction
  waveField->SetMeanWaveDirection(Direction(EAST(fc)), fc, dc);
//    waveField->SetDirectionalParameters(2,10);

  std::vector<double> phases_temp;
  phases_temp.push_back(0.);
  phases_temp.push_back(0.);
  std::vector<std::vector<double>> phases;
  phases.push_back(phases_temp);

  waveField->SetWavePhases(phases);

  waveField->Initialize();


  BiChromaticWaveInfDepth WaveRef(T1, T2, waveField->GetMeanWaveDirectionAngle(RAD, fc, dc), depth);

  // test at T = 0s
  double t = system.GetTime();
  //          test Elevation
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  EXPECT_NEAR(WaveRef.GetElevation(0., 1., t), waveField->GetElevation(0., 1., fc), 1e-8);
  EXPECT_NEAR(WaveRef.GetElevation(1., 0., t), waveField->GetElevation(1., 0., fc), 1e-8);

  //          test Velocity
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVx(0., 0., -1., t), waveField->GetVelocity(0., 0., -1., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., -1., t), waveField->GetVelocity(0., 0., -1., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., -1., t), waveField->GetVelocity(0., 0., -1., fc).GetVz(), 1e-8);

  //          test Acceleration
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAx(0., 0., -1., t), waveField->GetAcceleration(0., 0., -1., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., -1., t), waveField->GetAcceleration(0., 0., -1., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., -1., t), waveField->GetAcceleration(0., 0., -1., fc).GetAccZ(), 1e-8);


  // test at T = T2/2
  system.AdvanceOneStep(0.5 * T2);
  t = system.GetTime();
  //          test Elevation
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  //          test Velocity
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);

  // test at T = T2
  system.AdvanceOneStep(0.5 * T2);
  t = system.GetTime();
  //          test Elevation
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  //          test Velocity
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);

  // test at T = 3/2.T2
  system.AdvanceOneStep(0.5 * T2);
  t = system.GetTime();
  //          test Elevation
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  //          test Velocity
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);

  // test at T = T1s = 2*T2
  system.AdvanceOneStep(0.5 * T2);
  t = system.GetTime();
  //          test Elevation
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  //          test Velocity
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);


//    system.SetTimeStep(0.01);
//
//    system.Initialize();
//    system.RunInViewer(-10, 20, false);

}


struct BiDirectionalWaveInfDepth {
  double m_depth = 1000;
  double m_T = 6;
  double m_w = 2. * M_PI / m_T;;
  double m_k = SolveWaveDispersionRelation(m_depth, m_w, 9.81);
  double m_dir1 = 0;
  double m_dir2 = 0.1;
  double m_Amp = sqrt(2. * (m_dir2 - m_dir1));


  BiDirectionalWaveInfDepth(double T, double dir1, double dir2, double depth) {
    m_T = T;
    m_dir1 = dir1, m_dir2 = dir2;
    m_depth = depth;
    Init();
  }

  void Init() {
    m_w = 2. * M_PI / m_T;
    m_k = SolveWaveDispersionRelation(m_depth, m_w, 9.81);
    m_Amp = sqrt(2. * (m_dir2 - m_dir1));
  }

  double GetElevation(double x, double y, double t) {
    double kdir1 = x * cos(m_dir1) + y * sin(m_dir1);
    double kdir2 = x * cos(m_dir2) + y * sin(m_dir2);
    return m_Amp * (sin(m_k * kdir1 - m_w * t) + sin(m_k * kdir2 - m_w * t));
  }

  double GetPotential(double x, double y, double z, double t) {
    double kdir1 = x * cos(m_dir1) + y * sin(m_dir1);
    double kdir2 = x * cos(m_dir2) + y * sin(m_dir2);
    double g = 9.81;
    return -m_Amp * (g / m_w * cos(m_k * kdir1 - m_w * t) * exp(m_k * z)
                     + g / m_w * cos(m_k * kdir2 - m_w * t) * exp(m_k * z));
  }

  double GetVx(double x, double y, double z, double t) {
    double kdir1 = x * cos(m_dir1) + y * sin(m_dir1);
    double kdir2 = x * cos(m_dir2) + y * sin(m_dir2);
    return m_Amp * (m_w * cos(m_dir1) * sin(m_k * kdir1 - m_w * t) * exp(m_k * z)
                    + m_w * cos(m_dir2) * sin(m_k * kdir2 - m_w * t) * exp(m_k * z));
  }

  double GetVy(double x, double y, double z, double t) {
    double kdir1 = x * cos(m_dir1) + y * sin(m_dir1);
    double kdir2 = x * cos(m_dir2) + y * sin(m_dir2);
    return m_Amp * (m_w * sin(m_dir1) * sin(m_k * kdir1 - m_w * t) * exp(m_k * z)
                    + m_w * sin(m_dir2) * sin(m_k * kdir2 - m_w * t) * exp(m_k * z));
  }

  double GetVz(double x, double y, double z, double t) {
    double kdir1 = x * cos(m_dir1) + y * sin(m_dir1);
    double kdir2 = x * cos(m_dir2) + y * sin(m_dir2);
    return -m_Amp * (m_w * cos(m_k * kdir1 - m_w * t) * exp(m_k * z)
                     + m_w * cos(m_k * kdir2 - m_w * t) * exp(m_k * z));
  }

  double GetAx(double x, double y, double z, double t) {
    double kdir1 = x * cos(m_dir1) + y * sin(m_dir1);
    double kdir2 = x * cos(m_dir2) + y * sin(m_dir2);
    return -m_Amp * (m_w * m_w * cos(m_dir1) * cos(m_k * kdir1 - m_w * t) * exp(m_k * z)
                     + m_w * m_w * cos(m_dir2) * cos(m_k * kdir2 - m_w * t) * exp(m_k * z));
  }

  double GetAy(double x, double y, double z, double t) {
    double kdir1 = x * cos(m_dir1) + y * sin(m_dir1);
    double kdir2 = x * cos(m_dir2) + y * sin(m_dir2);
    return -m_Amp * (m_w * m_w * sin(m_dir1) * cos(m_k * kdir1 - m_w * t) * exp(m_k * z)
                     + m_w * m_w * sin(m_dir2) * cos(m_k * kdir2 - m_w * t) * exp(m_k * z));
  }

  double GetAz(double x, double y, double z, double t) {
    double kdir1 = x * cos(m_dir1) + y * sin(m_dir1);
    double kdir2 = x * cos(m_dir2) + y * sin(m_dir2);
    return -m_Amp * (m_w * m_w * sin(m_k * kdir1 - m_w * t) * exp(m_k * z)
                     + m_w * m_w * sin(m_k * kdir2 - m_w * t) * exp(m_k * z));
  }


};

TEST(FrWaveField, BiDirectionalWaveInfDepth) {
  double g = 9.81;

  // Frame convention
  FRAME_CONVENTION fc = NWU;
  // Wave direction convention
  DIRECTION_CONVENTION dc = GOTO;

  // create a system
  FrOffshoreSystem system("test_BiDirectionalWaveInfDepth");

  // Set depth to infinite
  system.GetEnvironment()->GetOcean()->SetInfiniteDepth();
  double depth = -1000;
//    system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(depth, fc);

  // Set the waveField to AiryRegular
  auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();
  auto waveField = freeSurface->SetAiryIrregularWaveField();

//    // Set the JONSWAP wave spectrum
//    double Hs = 3;
//    double Tp = 9;
//    auto Jonswap = waveField->SetJonswapWaveSpectrum(Hs, Tp);

  waveField->SetTestWaveSpectrum();
  double T = 12;
  auto w = 2. * M_PI / T;

  waveField->SetWaveFrequencies(w, w, 1);

  // Set wave direction
  waveField->SetMeanWaveDirection(Direction(NORTH(fc)), fc, dc);
  waveField->SetDirectionalParameters(2, 10, DIRTEST);

  std::vector<double> phases_temp;
  phases_temp.push_back(0.);
  std::vector<std::vector<double>> phases;
  phases.push_back(phases_temp);
  phases.push_back(phases_temp);

  waveField->SetWavePhases(phases);

  waveField->Initialize();


  double dir1 = 0, dir2 = 0.1;
  BiDirectionalWaveInfDepth WaveRef(T, dir1, dir2, depth);

  // test at T = 0s
  double t = system.GetTime();
  //          test Elevation
//    EXPECT_NEAR(0, waveField->GetElevation(0.,0.), 1e-8);
//    EXPECT_NEAR(0, waveField->GetElevation(0,1.), 1e-8);
  auto truc = waveField->GetElevation(0., 0., fc);
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  EXPECT_NEAR(WaveRef.GetElevation(0., 1., t), waveField->GetElevation(0., 1., fc), 1e-8);
  EXPECT_NEAR(WaveRef.GetElevation(1., 0., t), waveField->GetElevation(1., 0., fc), 1e-8);

  //          test Velocity
//    EXPECT_NEAR(0., waveField->GetVelocity(0.,0.,0.).GetVx(), 1e-8);
//    EXPECT_NEAR(0., waveField->GetVelocity(0.,0.,0.).GetVy(), 1e-8);
//    EXPECT_NEAR(Amp*(w1+w2), -waveField->GetVelocity(0.,0.,0.).GetVz(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVx(0., 0., -1., t), waveField->GetVelocity(0., 0., -1., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., -1., t), waveField->GetVelocity(0., 0., -1., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., -1., t), waveField->GetVelocity(0., 0., -1., fc).GetVz(), 1e-8);

  //          test Acceleration
//    EXPECT_NEAR(Amp*(w1*w1+w2*w2), -waveField->GetAcceleration(0.,0.,0.).GetAccX(), 1e-8);
//    EXPECT_NEAR(0., waveField->GetAcceleration(0.,0.,0.).GetAccY(), 1e-8);
//    EXPECT_NEAR(0., waveField->GetAcceleration(0.,0.,0.).GetAccZ(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAx(0., 0., -1., t), waveField->GetAcceleration(0., 0., -1., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., -1., t), waveField->GetAcceleration(0., 0., -1., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., -1., t), waveField->GetAcceleration(0., 0., -1., fc).GetAccZ(), 1e-8);


  // test at T = T2/2
  system.AdvanceOneStep(0.5 * T);
  t = system.GetTime();
//    EXPECT_DOUBLE_EQ(0.5*T2,t);
  //          test Elevation
//    EXPECT_NEAR(Amp, -waveField->GetElevation(0.,0.), 1e-8);
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  //          test Velocity
//    EXPECT_NEAR(Amp*w1, -waveField->GetVelocity(0.,0.,0.).GetVx(), 1e-8);
//    EXPECT_NEAR(Amp*w2, waveField->GetVelocity(0.,0.,0.).GetVz(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
//    EXPECT_NEAR(Amp*w2*w2, waveField->GetAcceleration(0.,0.,0.).GetAccX(), 1e-8);
//    EXPECT_NEAR(Amp*w1*w1, waveField->GetAcceleration(0.,0.,0.).GetAccZ(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);

  // test at T = T2
  system.AdvanceOneStep(0.5 * T);
  t = system.GetTime();
//    EXPECT_DOUBLE_EQ(T2,t);
  //          test Elevation
//    EXPECT_NEAR(0, waveField->GetElevation(0.,0.), 1e-8);
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  //          test Velocity
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);

  // test at T = 3/2.T2
  system.AdvanceOneStep(0.5 * T);
  t = system.GetTime();
//    EXPECT_DOUBLE_EQ(1.5*T2,t);
  //          test Elevation
//    EXPECT_NEAR(Amp, waveField->GetElevation(0.,0.), 1e-8);
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  //          test Velocity
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);

  // test at T = T1s = 2*T2
  system.AdvanceOneStep(0.5 * T);
  t = system.GetTime();
//    EXPECT_DOUBLE_EQ(T1,t);
  //          test Elevation
//    EXPECT_NEAR(0., waveField->GetElevation(0.,0.), 1e-8);
  EXPECT_NEAR(WaveRef.GetElevation(0., 0., t), waveField->GetElevation(0., 0., fc), 1e-8);
  //          test Velocity
  EXPECT_NEAR(WaveRef.GetVx(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVx(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVy(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVy(), 1e-8);
  EXPECT_NEAR(WaveRef.GetVz(0., 0., 0., t), waveField->GetVelocity(0., 0., 0., fc).GetVz(), 1e-8);
  //          test Acceleration
  EXPECT_NEAR(WaveRef.GetAx(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccX(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAy(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccY(), 1e-8);
  EXPECT_NEAR(WaveRef.GetAz(0., 0., 0., t), waveField->GetAcceleration(0., 0., 0., fc).GetAccZ(), 1e-8);


//    system.SetTimeStep(0.01);
//
//    system.Initialize();
//    system.RunInViewer(-10, 20, false);

}
