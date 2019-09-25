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

#include "FrLinearHDBForce.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"

namespace frydom {

    void FrLinearHDBForce::Initialize() {

      // Wave field.
      auto waveField = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

      // BEMBody.
      auto BEMBody = m_HDB->GetBody(m_body);

      // Interpolation of the excitation loads with respect to the wave direction.
      BuildHDBInterpolators();

      // Frequency and wave direction discretization.
      auto freqs = waveField->GetWaveFrequencies(RADS);
      auto directions = waveField->GetWaveDirections(RAD, NWU, GOTO);

      // Interpolation of the exciting loads if not already done.
      if (m_Fhdb.empty()) {
        m_Fhdb = GetHDBInterp(freqs, directions);
      }

      // Initialization of the parent class.
      FrForce::Initialize();

    }

    std::vector<Eigen::MatrixXcd>
    FrLinearHDBForce::GetHDBInterp(std::vector<double> waveFrequencies,
                                   std::vector<double> waveDirections) {

      // This function return the excitation force (linear excitation) or the diffraction force (nonlinear excitation) form the interpolator.

      // BEMBody.
      auto BEMBody = m_HDB->GetBody(m_body);

      // --> Getting sizes

      auto nbFreqInterp = waveFrequencies.size();
      auto nbFreqBDD = BEMBody->GetNbFrequencies();
      auto nbDirInterp = waveDirections.size();
      auto nbForceMode = BEMBody->GetNbForceMode();

      // Wave direction is expressed between 0 and 2*pi.
      for (auto &dir : waveDirections) dir = mathutils::Normalize_0_2PI(dir);

      std::vector<Eigen::MatrixXcd> Fexc;
      Fexc.reserve(nbDirInterp);

      // -> Building interpolator and return vector

      auto freqsBDD = std::make_shared<std::vector<double>>(BEMBody->GetFrequencies());

      auto freqCoeffs = std::make_shared<std::vector<std::complex<double>>>();
      freqCoeffs->reserve(nbFreqBDD);

      for (auto direction: waveDirections) {

        auto excitationForceDir = Eigen::MatrixXcd(nbForceMode, nbFreqInterp);

        for (unsigned int imode = 0; imode < nbForceMode; ++imode) {

          freqCoeffs->clear();
          for (unsigned int ifreq = 0; ifreq < nbFreqBDD; ++ifreq) {
            freqCoeffs->push_back(m_waveDirInterpolators[imode][ifreq](direction));
          }

          auto freqInterpolator = mathutils::Interp1dLinear<double, std::complex<double>>();
          freqInterpolator.Initialize(freqsBDD, freqCoeffs);

          auto freqCoeffsInterp = freqInterpolator(waveFrequencies);
          for (unsigned int ifreq = 0; ifreq < nbFreqInterp; ++ifreq) {
            excitationForceDir(imode, ifreq) = freqCoeffsInterp[ifreq];
          }
        }
        Fexc.push_back(excitationForceDir);
      }
      return Fexc;
    }

    void FrLinearHDBForce::BuildHDBInterpolators() {

      // This function creates the interpolator for the excitation loads (linear excitation) or the diffraction loads (nonlinear excitation) with respect to the wave frequencies and directions.

      // BEMBody.
      auto BEMBody = m_HDB->GetBody(m_body);

      auto nbWaveDirections = BEMBody->GetNbWaveDirections();
      auto nbFreq = BEMBody->GetNbFrequencies();
      auto nbForceModes = BEMBody->GetNbForceMode();

      m_waveDirInterpolators.clear();
      m_waveDirInterpolators.reserve(nbForceModes);

      auto angles = std::make_shared<std::vector<double>>(BEMBody->GetWaveDirections(mathutils::RAD, NWU));

      auto interpolators = std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>();
      interpolators.reserve(nbFreq);

      for (unsigned int imode = 0; imode < nbForceModes; ++imode) {

        interpolators.clear();

        for (unsigned int ifreq = 0; ifreq < nbFreq; ++ifreq) {

          auto coeffs = std::make_shared<std::vector<std::complex<double>>>();
          coeffs->reserve(nbWaveDirections);

          for (unsigned int iangle = 0; iangle < nbWaveDirections; ++iangle) {
            auto data = GetHDBData(iangle);
            coeffs->push_back(data(imode, ifreq));
          }

          auto interpolator = mathutils::Interp1dLinear<double, std::complex<double>>();
          interpolator.Initialize(angles, coeffs);
          interpolators.push_back(interpolator);
        }
        m_waveDirInterpolators.push_back(interpolators);
      }
    }

    void FrLinearHDBForce::Compute_F_HDB() {

      // This function computes the excitation loads (linear excitation) or the diffraction loads (nonlinear excitation).

      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(m_body);

      // Wave field structure.
      auto waveField = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

      // Wave elevation.
      auto complexElevations = waveField->GetComplexElevation(eqFrame->GetFrameInWorld().GetX(NWU),
                                                              eqFrame->GetFrameInWorld().GetY(NWU),
                                                              NWU);

      // DOF.
      auto nbMode = m_HDB->GetBody(m_body)->GetNbForceMode();

      // Number of wave frequencies.
      auto nbFreq = waveField->GetWaveFrequencies(RADS).size();

      // Number of wave directions.
      auto nbWaveDir = waveField->GetWaveDirections(RAD, NWU, GOTO).size();

      // Fexc(t) = eta*Fexc(Nemoh).
      Eigen::VectorXd forceMode(nbMode);
      forceMode.setZero(); // Initialization.
      for (unsigned int imode = 0; imode < nbMode; ++imode) {
        for (unsigned int ifreq = 0; ifreq < nbFreq; ++ifreq) {
          for (unsigned int idir = 0; idir < nbWaveDir; ++idir) {
            forceMode(imode) += std::imag(complexElevations[idir][ifreq] * m_Fhdb[idir](imode, ifreq));
          }
        }
      }

      // From vector to force and torque structures.
      Force force;
      force.SetNull();
      Torque torque;
      torque.SetNull();

      for (unsigned int imode = 0; imode < nbMode; ++imode) {

        auto mode = m_HDB->GetBody(m_body)->GetForceMode(imode);
        Direction direction = mode->GetDirection(); // Unit vector for the force direction.
        switch (mode->GetType()) {
          case FrBEMMode::LINEAR:
            force += direction * forceMode(imode);
            break;
          case FrBEMMode::ANGULAR:
            torque += direction * forceMode(imode);
            break;
        }
      }

      // Projection of the loads in the equilibrium frame.
      auto forceInWorld = eqFrame->GetFrameInWorld().ProjectVectorFrameInParent(force, NWU);
      auto torqueInWorldAtCOG = eqFrame->GetFrameInWorld().ProjectVectorFrameInParent(torque, NWU);

      // Setting the nonlinear excitation loads in world at the CoG in world.
      SetForceTorqueInWorldAtCOG(forceInWorld, torqueInWorldAtCOG, NWU);

    }

    void FrLinearHDBForce::Compute(double time) {
      Compute_F_HDB();
    }

    FrLinearHDBForce::FrLinearHDBForce(const std::string &&name, const std::shared_ptr<FrHydroDB> &HDB) :
        FrForce(std::move(name)), m_HDB(HDB) {}


} // end namespace frydom
