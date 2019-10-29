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

#include <random>
#include <frydom/logging/FrEventLogger.h>

#include "FrAiryIrregularWaveField.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveDispersionRelation.h"

#include "MathUtils/VectorGeneration.h"
#include "MathUtils/Angles.h"

namespace frydom {

  FrAiryIrregularWaveField::FrAiryIrregularWaveField(FrFreeSurface *freeSurface) : FrWaveField(freeSurface) {
    m_waveModel = LINEAR_WAVES;
    m_verticalFactor = std::make_unique<FrKinematicStretching>();

    m_waveSpectrum = std::make_unique<FrJonswapWaveSpectrum>();
  }

  void FrAiryIrregularWaveField::SetWaveFrequencies(double minFreq, double maxFreq, unsigned int nbFreq) {
    assert(minFreq > 0);
    assert(minFreq <= maxFreq);
    assert(nbFreq != 0);
    m_minFreq = minFreq;
    m_maxFreq = maxFreq;
    m_nbFreq = nbFreq;

    m_waveFrequencies.clear();
    if (nbFreq == 1) {
      assert(minFreq == maxFreq);
      m_waveFrequencies.push_back(m_minFreq);
    } else {
      m_waveFrequencies = mathutils::linspace(m_minFreq, m_maxFreq, m_nbFreq);
    }

    // Caching wave numbers
    m_waveNumbers.clear();

    // Set the wave numbers, using the wave dispersion relation
    auto gravityAcceleration = m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();

    double k;
    if (m_infinite_depth) {
      m_waveNumbers.reserve(m_nbFreq);
      for (auto freq:m_waveFrequencies) {
        k = pow(freq, 2) / gravityAcceleration;
        m_waveNumbers.push_back(k);
      }
    } else {
      m_waveNumbers = SolveWaveDispersionRelation(m_freeSurface->GetOcean()->GetDepth(NWU), m_waveFrequencies,
                                                  gravityAcceleration);
    }
    if (!m_waveDirections.empty()) {
      c_amplitude = m_waveSpectrum->GetWaveAmplitudes(m_waveFrequencies, m_waveDirections);
    }

    event_logger::info("IrregularWaveField", "", "Wave frequencies set to (rad/s) wmin = {}, wmax = {}, nbFreq = {}",
                       minFreq, maxFreq, nbFreq);

  }

  void FrAiryIrregularWaveField::SetMeanWaveDirectionAngle(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                           DIRECTION_CONVENTION dc) {
    // The wave direction angle is used internally with the convention NWU, GOTO, and RAD unit.
    m_meanDir = dirAngle;
    if (unit == mathutils::DEG) m_meanDir *= DEG2RAD;
    if (IsNED(fc)) m_meanDir = -m_meanDir;
    if (IsCOMEFROM(dc)) m_meanDir -= MU_PI;

    m_meanDir = mathutils::Normalize_0_2PI(m_meanDir);

    event_logger::info("IrregularWaveField", "", "Mean wave direction set to {} deg in NED and GOTO conventions",
                       GetMeanWaveDirection(NED, GOTO));

    if (m_waveSpectrum->GetDirectionalModel() != nullptr) { ComputeWaveDirections(); }
  }

  void
  FrAiryIrregularWaveField::SetMeanWaveDirection(Direction direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
    assert(mathutils::IsClose(direction.Getuz(), 0.));
    double dirAngle = atan2(direction.Getuy(), direction.Getux());
    SetMeanWaveDirectionAngle(dirAngle, mathutils::RAD, fc, dc);
  }

  void FrAiryIrregularWaveField::SetDirectionalParameters(unsigned int nbDir, double spreadingFactor,
                                                          WAVE_DIRECTIONAL_MODEL dirType) {
    m_nbDir = nbDir;
    m_waveDirections.clear();
    switch (dirType) {
      case NONE:
        m_waveSpectrum->SetDirectionalModel(dirType);
        break;
      case DIRTEST:
        m_waveSpectrum->SetDirectionalModel(dirType);
        break;
      case COS2S:
        m_waveSpectrum->SetCos2sDirectionalModel(spreadingFactor);
        break;
    }

    event_logger::info("IrregularWaveField", "",
                       "Directional parameters have been set with nb_dir = {} and spreading_factor = {}",
                       nbDir, spreadingFactor);

    ComputeWaveDirections();
  }

  void FrAiryIrregularWaveField::ComputeWaveDirections() {

    assert(m_waveSpectrum != nullptr);

    double spreadingFactor, dirBounds, BoundParameter;
    double theta_min, theta_max;

    m_waveDirections.clear();
    switch (m_waveSpectrum->GetDirectionalModel()->GetType()) {
      case NONE:
        m_waveDirections.push_back(m_meanDir);
        break;
      case COS2S:
        m_waveSpectrum->GetDirectionalModel()->GetDirectionBandwidth(theta_min, theta_max, m_meanDir);
        m_waveDirections = mathutils::linspace(theta_min, theta_max, m_nbDir);
        break;
      case DIRTEST:
        m_waveDirections = mathutils::linspace(m_meanDir, m_meanDir + 0.1, m_nbDir);
        break;
    }

//        for (auto& dir:m_waveDirections) {dir = mathutils::Normalize_0_2PI(dir);};

    if (!m_waveFrequencies.empty()) {
      c_amplitude = m_waveSpectrum->GetWaveAmplitudes(m_waveFrequencies, m_waveDirections);
    }
  }

  void FrAiryIrregularWaveField::SetWavePhases(std::vector<std::vector<double>> &wavePhases) {
    assert(wavePhases.size() == m_nbDir);
    for (auto &w: wavePhases) {
      assert(w.size() == m_nbFreq);
    }
    m_wavePhases = std::make_unique<std::vector<std::vector<double>>>(wavePhases);
    event_logger::info("IrregularWaveField", "", "Phases have been set with external values");
  }

  FrJonswapWaveSpectrum *FrAiryIrregularWaveField::SetJonswapWaveSpectrum(double Hs, double Tp, double gamma) {
    m_waveSpectrum = std::make_unique<FrJonswapWaveSpectrum>(Hs, Tp, gamma);
    event_logger::info("IrregularWaveField", "",
                       "Wave Spectrum has been set to JONSWAP(Hs {} m, Tp {}, gamma {})", Hs, Tp, gamma);
    return dynamic_cast<FrJonswapWaveSpectrum *>(m_waveSpectrum.get());
  }

  FrPiersonMoskowitzWaveSpectrum *FrAiryIrregularWaveField::SetPiersonMoskovitzWaveSpectrum(double Hs, double Tp) {
    m_waveSpectrum = std::make_unique<FrPiersonMoskowitzWaveSpectrum>(Hs, Tp);
    event_logger::info("IrregularWaveField", "",
                       "Wave Spectrum has been set to PIERSON_MOSKOWITZ(Hs {} m, Tp {})", Hs, Tp);
    return dynamic_cast<FrPiersonMoskowitzWaveSpectrum *>(m_waveSpectrum.get());
  }

  FrTestWaveSpectrum *FrAiryIrregularWaveField::SetTestWaveSpectrum() {
    m_waveSpectrum = std::make_unique<FrTestWaveSpectrum>();
    return dynamic_cast<FrTestWaveSpectrum *>(m_waveSpectrum.get());
  }

  FrWaveSpectrum *FrAiryIrregularWaveField::GetWaveSpectrum() const { return m_waveSpectrum.get(); }

  void FrAiryIrregularWaveField::SetStretching(STRETCHING_TYPE type) {
    switch (type) {
      case NO_STRETCHING:
        m_verticalFactor = std::make_unique<FrKinematicStretching>();
        event_logger::info("IrregularWaveField", "Kinematic Stretching", "No stretching");
        break;
      case VERTICAL:
        m_verticalFactor = std::make_unique<FrKinStretchingVertical>();
        event_logger::info("IrregularWaveField", "Kinematic Stretching", "Vertical stretching");
        break;
      case EXTRAPOLATE:
        m_verticalFactor = std::make_unique<FrKinStretchingExtrapol>();
        event_logger::info("IrregularWaveField", "Kinematic Stretching", "Extrapolation stretching");
        break;
      case WHEELER:
        m_verticalFactor = std::make_unique<FrKinStretchingWheeler>(this);
        event_logger::info("IrregularWaveField", "Kinematic Stretching", "Wheeler stretching");
        break;
      case CHAKRABARTI:
        m_verticalFactor = std::make_unique<FrKinStretchingChakrabarti>(this);
        event_logger::info("IrregularWaveField", "Kinematic Stretching", "Chakrabarti stretching");
        break;
      case DELTA:
        m_verticalFactor = std::make_unique<FrKinStretchingDelta>(this);
        event_logger::info("IrregularWaveField", "Kinematic Stretching", "Delta stretching");
        break;
      case HDELTA:
        m_verticalFactor = std::make_unique<FrKinStretchingHDelta>(this);
        event_logger::info("IrregularWaveField", "Kinematic Stretching", "H Delta stretching");
        break;
      default:
        m_verticalFactor = std::make_unique<FrKinematicStretching>();
        event_logger::info("IrregularWaveField", "Kinematic Stretching", "No stretching");
        break;
    }
    m_verticalFactor->SetInfDepth(m_infinite_depth);
  }

  void FrAiryIrregularWaveField::GenerateRandomWavePhases() {

    std::random_device rd;
    std::mt19937 gen(rd());
    GenerateRandomWavePhases(gen);
  }

  void FrAiryIrregularWaveField::GenerateRandomWavePhases(int seed) {

    std::mt19937 gen(seed);
    GenerateRandomWavePhases(gen);

  }

  void FrAiryIrregularWaveField::GenerateRandomWavePhases(std::mt19937 &seed) {

    m_wavePhases = std::make_unique<std::vector<std::vector<double>>>();
    m_wavePhases->clear();
    m_wavePhases->reserve(m_waveDirections.size());

    std::vector<double> phases;
    phases.reserve(m_waveFrequencies.size());

    std::uniform_real_distribution<double> dis(0., MU_2PI);

    for (double dir: m_waveDirections) {
      phases.clear();
      for (double freq: m_waveFrequencies) {
        phases.push_back(dis(seed));
      }
      m_wavePhases->push_back(phases);
    }
    event_logger::info("IrregularWaveField", "", "Random phases generated");

  }

  std::vector<double> FrAiryIrregularWaveField::GetWaveFrequencies(FREQUENCY_UNIT unit) const {
    std::vector<double> freqs = m_waveFrequencies;
    if (unit != mathutils::RADS) {
      for (auto &freq: freqs) {
        freq = convert_frequency(freq, mathutils::RADS, unit);
      }
    }
    return freqs;
  }

  std::vector<double> FrAiryIrregularWaveField::GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                                  DIRECTION_CONVENTION dc) const {
    auto directions = m_waveDirections;

    if (IsNED(fc)) for (auto &dir: directions) { dir = -dir; }
    if (dc == COMEFROM) for (auto &dir: directions) { dir += M_PI; }
    for (auto &dir: directions) mathutils::Normalize_0_2PI(dir);
    if (unit == mathutils::DEG) for (auto &dir: directions) { dir *= MU_180_PI; }

    return directions;

  }

  void FrAiryIrregularWaveField::Initialize() {
    event_logger::info("WaveField", "AiryIrregularOptimWaveField",
                       "Airy Irregular Wave field initialization with {} wave directions "
                       "and {} wave frequencies", m_nbDir, m_nbFreq);
    event_logger::flush();

    FrWaveField::Initialize();
    m_verticalFactor->SetInfDepth(m_infinite_depth);

    if (m_waveDirections.empty()) { m_waveDirections.push_back(m_meanDir); }

    if (m_minFreq == 0. && m_maxFreq == 0.) {
      GetWaveSpectrum()->GetFrequencyBandwidth(m_minFreq, m_maxFreq);
    }

    // Initialize wave frequency vector
    SetWaveFrequencies(m_minFreq, m_maxFreq, m_nbFreq);

    // Checks wave phases, and randomly generate them if needed
    bool testSize = true;
    if (m_wavePhases != nullptr) {
      testSize = m_wavePhases->size() == m_nbDir;
      for (unsigned int idir = 0; idir < m_nbDir; ++idir) {
        testSize = testSize & m_wavePhases->at(idir).size() == m_nbFreq;
      }
    }

    if (m_wavePhases == nullptr || !testSize) {
      GenerateRandomWavePhases();
    }

    Update(0.);
  }

  std::vector<std::vector<Complex>>
  FrAiryIrregularWaveField::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {

    // This function gets the complex wave elevation at the position (x,y,0), of the irregular Airy wave field.

    // Frame convention.
    double NWUsign = 1;
    if (IsNED(fc)) {
      y = -y;
      NWUsign = -NWUsign;
    }

    // Data structure to store the complex elevation for every frequency and every wave direction.
    std::vector<std::vector<Complex>> ComplexElevation;
    ComplexElevation.reserve(m_nbDir);
    ComplexElevation.clear();

    // Temporary data structure to store the complex elevation for every frequency.
    std::vector<Complex> ComplexElevation_temp;
    ComplexElevation_temp.reserve(m_nbFreq);
    ComplexElevation_temp.clear();

    double aik, ki, phi_ik, kdir;
    Complex elevation;

    std::vector<double> amplitudeTemp;

    // Loop over the wave direction.
    for (unsigned int idir = 0; idir < m_nbDir; ++idir) {

      kdir = x * cos(m_waveDirections[idir]) + y * sin(m_waveDirections[idir]);
      amplitudeTemp = c_amplitude[idir];
      ComplexElevation_temp.clear();

      // Loop over the frequency.
      for (unsigned int ifreq = 0; ifreq < m_nbFreq; ++ifreq) {
        ki = m_waveNumbers[ifreq];
        aik = amplitudeTemp[ifreq];
        phi_ik = m_wavePhases->at(idir)[ifreq];
        elevation = aik * exp(JJ * (ki * kdir - m_waveFrequencies[ifreq] * c_time + phi_ik)) * NWUsign * c_ramp;
        ComplexElevation_temp.push_back(elevation);
      }

      ComplexElevation.push_back(ComplexElevation_temp);
    }

    return ComplexElevation;
  }

  std::vector<mathutils::Vector3d<Complex>>
  FrAiryIrregularWaveField::GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
    double NWUsign = 1;
    if (IsNED(fc)) {
      y = -y;
      z = -z;
      NWUsign = -NWUsign;
    }

    std::vector<mathutils::Vector3d<Complex>> ComplexVel;
    ComplexVel.reserve(m_nbFreq);
    ComplexVel.clear();

    Complex Vx = 0, Vy = 0, Vz = 0;
    double ki, wi, thetaj;
    double Stretching, StretchingDZ;

    auto ComplexElevation = GetComplexElevation(x, y, fc);

    for (unsigned int ifreq = 0; ifreq < m_nbFreq; ++ifreq) {
      ki = m_waveNumbers[ifreq];
      wi = m_waveFrequencies[ifreq];
      Vx = 0, Vy = 0, Vz = 0;
      Stretching = m_verticalFactor->Eval(x, y, z, ki, c_depth);
      StretchingDZ = m_verticalFactor->EvalDZ(x, y, z, ki, c_depth);
      for (unsigned int idir = 0; idir < m_nbDir; ++idir) {
        thetaj = m_waveDirections[idir];
        Vx += cos(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching * NWUsign;
        Vy += sin(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching;
        Vz += -JJ / ki * wi * ComplexElevation[idir][ifreq] * StretchingDZ;
      }
      ComplexVel.emplace_back(Vx, Vy, Vz);
    }

    return ComplexVel;
  }

  double FrAiryIrregularWaveField::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
    double elevation = 0.;

    auto ComplexElevation = GetComplexElevation(x, y, fc);

    for (unsigned int idir = 0; idir < m_nbDir; ++idir) {
      for (unsigned int ifreq = 0; ifreq < m_nbFreq; ++ifreq) {
        elevation += std::imag(ComplexElevation[idir][ifreq]);
      }
    }
    return elevation;
  }

  Velocity FrAiryIrregularWaveField::GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
    Velocity Vel = {0., 0., 0.};
    auto cplxVel = GetComplexVelocity(x, y, z, fc);
    for (unsigned int ifreq = 0; ifreq < m_nbFreq; ++ifreq) {
      Vel.GetVx() += std::imag(cplxVel[ifreq].x());
      Vel.GetVy() += std::imag(cplxVel[ifreq].y());
      Vel.GetVz() += std::imag(cplxVel[ifreq].z());
    }
    return Vel;
  }

  Acceleration FrAiryIrregularWaveField::GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const {
    Acceleration Acc = {0., 0., 0.};
    auto cplxVel = GetComplexVelocity(x, y, z, fc);
    double wi;
    for (unsigned int ifreq = 0; ifreq < m_nbFreq; ++ifreq) {
      wi = m_waveFrequencies[ifreq];
      Acc.GetAccX() += std::imag(-JJ * wi * cplxVel[ifreq].x());
      Acc.GetAccY() += std::imag(-JJ * wi * cplxVel[ifreq].y());
      Acc.GetAccZ() += std::imag(-JJ * wi * cplxVel[ifreq].z());
    }
    return Acc;
  }

  double FrAiryIrregularWaveField::GetMeanWaveDirectionAngle(ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                             DIRECTION_CONVENTION dc) const {
    double dirAngle = m_meanDir;
    if (IsNED(fc)) dirAngle = -dirAngle;
    if (IsCOMEFROM(dc)) dirAngle -= MU_PI;
    if (unit == mathutils::DEG) dirAngle *= RAD2DEG;

    return mathutils::Normalize_0_360(dirAngle);
  }

  Direction FrAiryIrregularWaveField::GetMeanWaveDirection(FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const {
    auto dirAngle = GetMeanWaveDirectionAngle(mathutils::RAD, fc, dc);
    return {cos(dirAngle), sin(dirAngle), 0.};
  }

  // ------------------------------------- Pressure ----------------------------

  double FrAiryIrregularWaveField::GetPressure(double x, double y, double z, FRAME_CONVENTION fc) const {

    // This function gets the pressure at the position (x,y,z) for a irregular Airy wave field.

    double Pressure = 0;

    // Frame convention.
    double NWUsign = 1;
    if (IsNED(fc)) {
      y = -y;
      NWUsign = -NWUsign;
    }

    double aik, ki, phi_ik;
    std::vector<double> amplitudeTemp;
    double kdir, th, Ez;

    // Loop over the frequencies.
    for (unsigned int ifreq = 0; ifreq < m_nbFreq; ++ifreq) { // n.
      ki = m_waveNumbers[ifreq];
      if (m_infinite_depth) { // Infinite water depth.
        th = 1;
      } else { // Finite water depth.
        th = tanh(ki * c_depth);
      }
      Ez = m_verticalFactor->Eval(x, y, z, ki, c_depth);
      // Loop over the wave directions.
      for (unsigned int idir = 0; idir < m_nbDir; ++idir) { // m.
        kdir = x * cos(m_waveDirections[idir]) + y * sin(m_waveDirections[idir]);
        aik = c_amplitude[idir][ifreq];
        phi_ik = m_wavePhases->at(idir)[ifreq];
        Pressure = Pressure + Ez * th * std::imag(
            aik * exp(JJ * (ki * kdir - m_waveFrequencies[ifreq] * c_time + phi_ik)) * NWUsign * c_ramp);
      }
    }

    // Rho*gravity term.
    Pressure = Pressure * c_density * c_gravity;

    return Pressure;

  }

}  // end namespace frydom
