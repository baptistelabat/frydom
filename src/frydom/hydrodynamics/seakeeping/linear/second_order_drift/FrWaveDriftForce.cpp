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


#include "FrWaveDriftForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"


namespace frydom {

  FrWaveDriftForce::FrWaveDriftForce(const std::string &name, FrBody *body, std::shared_ptr<FrHydroDB> hdb)
      : FrForce(name, body), m_hdb(hdb) {}

  void FrWaveDriftForce::Compute(double time) {

    auto force = Force();
    auto torque = Torque();

    auto body = GetBody();

    auto ocean = body->GetSystem()->GetEnvironment()->GetOcean();
    auto waveAmplitude = ocean->GetFreeSurface()->GetWaveField()->GetWaveAmplitudes();

    // Wave encounter frequencies
    auto eqFrame = m_hdb->GetMapper()->GetEquilibriumFrame(body);
    auto waveFrequencies = GetEncounterWaveFrequencies(eqFrame->GetVelocityInWorld(NWU));
    auto nbFreq = waveFrequencies[0].size();

    // Wave direction
    auto waveDir = GetRelativeWaveDir();
    auto nbWaveDir = waveDir.size();

    // Compute Wave drift force
    if (m_table->HasSurge()) {
      for (unsigned int idir = 0; idir < nbWaveDir; idir++) {
        auto angle = waveDir[idir];
        for (unsigned int ifreq = 0; ifreq < nbFreq; ifreq++) {
          auto freq = waveFrequencies[idir][ifreq];
          force.GetFx() += std::pow(waveAmplitude[idir][ifreq], 2) * m_table->Eval("surge", angle, freq);
        }
      }
    }

    if (m_table->HasSway()) {
      for (unsigned int idir = 0; idir < nbWaveDir; idir++) {
        auto angle = waveDir[idir];
        for (unsigned int ifreq = 0; ifreq < nbFreq; ifreq++) {
          auto freq = waveFrequencies[idir][ifreq];
          force.GetFy() += std::pow(waveAmplitude[idir][ifreq], 2) * m_table->Eval("sway", angle, freq);
        }
      }
    }

    if (m_table->HasHeave()) {
      for (unsigned int idir = 0; idir < nbWaveDir; idir++) {
        auto angle = waveDir[idir];
        for (unsigned int ifreq = 0; ifreq < nbFreq; ifreq++) {
          auto freq = waveFrequencies[idir][ifreq];
          force.GetFz() += std::pow(waveAmplitude[idir][ifreq], 2) * m_table->Eval("heave", angle, freq);
        }
      }
    }

    if (m_table->HasRoll()) {
      for (unsigned int idir = 0; idir < nbWaveDir; idir++) {
        auto angle = waveDir[idir];
        for (unsigned int ifreq = 0; ifreq < nbFreq; ifreq++) {
          auto freq = waveFrequencies[idir][ifreq];
          torque.GetMx() += std::pow(waveAmplitude[idir][ifreq], 2) * m_table->Eval("roll", angle, freq);
        }
      }
    }

    if (m_table->HasPitch()) {
      for (unsigned int idir = 0; idir < nbWaveDir; idir++) {
        auto angle = waveDir[idir];
        for (unsigned int ifreq = 0; ifreq < nbFreq; ifreq++) {
          auto freq = waveFrequencies[idir][ifreq];
          torque.GetMy() += std::pow(waveAmplitude[idir][ifreq], 2) * m_table->Eval("pitch", angle, freq);
        }
      }
    }

    if (m_table->HasYaw()) {
      for (unsigned int idir = 0; idir < nbWaveDir; idir++) {
        auto angle = waveDir[idir];
        for (unsigned int ifreq = 0; ifreq < nbFreq; ifreq++) {
          auto freq = waveFrequencies[idir][ifreq];
          torque.GetMz() += std::pow(waveAmplitude[idir][ifreq], 2) * m_table->Eval("yaw", angle, freq);
        }
      }
    }

    SetForceTorqueInBodyAtCOG(force, torque, NWU);
  }

  void FrWaveDriftForce::Initialize() {
    FrForce::Initialize();
    m_table = m_hdb->GetBody(GetBody())->GetWaveDrift();
  }

  std::vector<double> FrWaveDriftForce::GetRelativeWaveDir() const {

    auto body = GetBody();

    auto ocean = body->GetSystem()->GetEnvironment()->GetOcean();
    auto waveDir = ocean->GetFreeSurface()->GetWaveField()->GetWaveDirections(RAD, NWU, GOTO);

    auto BEMBody = m_hdb->GetBody(body);
    auto eqFrame = m_hdb->GetMapper()->GetEquilibriumFrame(BEMBody);

    double phi, theta, psi;
    eqFrame->GetFrameInWorld().GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);

    for (auto &val: waveDir) { val -= psi; }

    return waveDir;
  }

  std::vector<std::vector<double>> FrWaveDriftForce::GetEncounterWaveFrequencies(Velocity speed) const {

    auto waveField = GetBody()->GetSystem()->GetEnvironment()->GetOcean()
        ->GetFreeSurface()->GetWaveField();

    auto waveDir = waveField->GetWaveDirections(RAD, NWU, GOTO);
    auto nbDir = waveDir.size();

    // Get the speed component aligned with wave directions
    std::vector<double> velocity;
    for (auto &dir: waveDir) {
      auto vect = Direction(cos(dir), sin(dir), 0.);
      velocity.push_back(speed.dot(vect));
    }

    // Encounter frequencies
    std::vector<std::vector<double>> waveEncounterFrequencies;

    auto waveFrequencies = waveField->GetWaveFrequencies(RADS);
    auto waveNumbers = waveField->GetWaveNumbers();
    auto nbFreq = waveFrequencies.size();

    waveEncounterFrequencies.reserve(nbDir);
    for (unsigned int idir = 0; idir < nbDir; idir++) {
      std::vector<double> freqDir;
      freqDir.reserve(nbFreq);
      for (unsigned int ifreq = 0; ifreq < nbFreq; ifreq++) {
        freqDir.push_back(waveFrequencies[ifreq] - waveNumbers[ifreq] * velocity[idir]);
      }
      waveEncounterFrequencies.push_back(freqDir);
    }

    return waveEncounterFrequencies;

  }

}  // end namespace frydom
