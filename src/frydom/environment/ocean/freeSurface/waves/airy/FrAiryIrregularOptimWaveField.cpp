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


#include "FrAiryIrregularOptimWaveField.h"
#include "frydom/core/math/FrComplex.h"

#include <vector>

namespace frydom {

    template<class OffshoreSystemType, class StretchingType, class WaveSpectrumType>
    FrAiryIrregularOptimWaveField<OffshoreSystemType, StretchingType, WaveSpectrumType>::FrAiryIrregularOptimWaveField(
        FrFreeSurface<OffshoreSystemType> *freeSurface)
        : FrAiryIrregularWaveField<OffshoreSystemType, StretchingType, WaveSpectrumType>(freeSurface) {}

    template<class OffshoreSystemType, class StretchingType, class WaveSpectrumType>
    std::vector<std::vector<Complex>>
    FrAiryIrregularOptimWaveField<OffshoreSystemType, StretchingType, WaveSpectrumType>::GetComplexElevation(double x,
                                                                                                             double y,
                                                                                                             FRAME_CONVENTION fc) const {
      double NWUsign = 1;
      if (IsNED(fc)) {
        y = -y;
        NWUsign = -NWUsign;
      }

      std::vector<std::vector<Complex>> ComplexElevation;
      ComplexElevation.reserve(this->m_nbDir);
      ComplexElevation.clear();

      std::vector<Complex> ComplexElevation_temp;
      ComplexElevation_temp.reserve(this->m_nbFreq);
      ComplexElevation_temp.clear();

      double ki;
      Complex elevation;

      for (unsigned int idir = 0; idir < this->m_nbDir; ++idir) {
        double kdir = x * c_cosTheta[idir] + y * c_sinTheta[idir];
        ComplexElevation_temp.clear();
        for (unsigned int ifreq = 0; ifreq < this->m_nbFreq; ++ifreq) {
          ki = this->m_waveNumbers[ifreq];
          elevation = exp(JJ * ki * kdir) * c_expJwt[ifreq] * c_AExpJphi[idir][ifreq] * NWUsign * this->c_ramp;
          ComplexElevation_temp.push_back(elevation);
        }
        ComplexElevation.push_back(ComplexElevation_temp);
      }

      return ComplexElevation;
    }

    template<class OffshoreSystemType, class StretchingType, class WaveSpectrumType>
    std::vector<mathutils::Vector3d<Complex>>
    FrAiryIrregularOptimWaveField<OffshoreSystemType, StretchingType, WaveSpectrumType>::GetComplexVelocity(double x,
                                                                                                            double y,
                                                                                                            double z,
                                                                                                            FRAME_CONVENTION fc) const {
      double NWUsign = 1;
      if (IsNED(fc)) {
        y = -y;
        z = -z;
        NWUsign = -NWUsign;
      }
      std::vector<mathutils::Vector3d<Complex>> ComplexVel;
      ComplexVel.reserve(this->m_nbFreq);
      ComplexVel.clear();

      Complex Vx = 0, Vy = 0, Vz = 0;
      double ki, wi;
      double Stretching, StretchingDZ;

      auto ComplexElevation = GetComplexElevation(x, y, fc);

      for (unsigned int ifreq = 0; ifreq < this->m_nbFreq; ++ifreq) {
        ki = this->m_waveNumbers[ifreq];
        wi = this->m_waveFrequencies[ifreq];
        Vx = 0, Vy = 0, Vz = 0;
        Stretching = this->m_verticalFactor->Eval(x, y, z, ki, this->c_depth);
        StretchingDZ = this->m_verticalFactor->EvalDZ(x, y, z, ki, this->c_depth);
        for (unsigned int idir = 0; idir < this->m_nbDir; ++idir) {
          Vx += c_cosTheta[idir] * wi * ComplexElevation[idir][ifreq] * Stretching * NWUsign;
          Vy += c_sinTheta[idir] * wi * ComplexElevation[idir][ifreq] * Stretching;
          Vz += -JJ / ki * wi * ComplexElevation[idir][ifreq] * StretchingDZ;
        }
        ComplexVel.emplace_back(Vx, Vy, Vz);
      }

      return ComplexVel;
    }

    template<class OffshoreSystemType, class StretchingType, class WaveSpectrumType>
    void FrAiryIrregularOptimWaveField<OffshoreSystemType, StretchingType, WaveSpectrumType>::Initialize() {
      FrAiryIrregularWaveField<OffshoreSystemType, StretchingType, WaveSpectrumType>::Initialize();

      c_cosTheta.reserve(this->m_nbDir);
      c_cosTheta.clear();
      c_sinTheta.reserve(this->m_nbDir);
      c_sinTheta.clear();
      c_AExpJphi.reserve(this->m_nbDir);
      c_AExpJphi.clear();

      Complex expTemp;
      double dirTemp;

      std::vector<Complex> Aphi_temp;
      Aphi_temp.reserve(this->m_nbFreq);
      Aphi_temp.clear();

      auto Amplitudes = this->m_waveSpectrum->GetWaveAmplitudes(this->m_waveFrequencies, this->m_waveDirections);

      for (unsigned int idir = 0; idir < this->m_nbDir; ++idir) {
        dirTemp = cos(this->m_waveDirections[idir]);
        c_cosTheta.push_back(dirTemp);
        dirTemp = sin(this->m_waveDirections[idir]);
        c_sinTheta.push_back(dirTemp);
        Aphi_temp.clear();
        for (unsigned int ifreq = 0; ifreq < this->m_nbFreq; ++ifreq) {
          expTemp = Amplitudes[idir][ifreq] * exp(JJ * this->m_wavePhases->at(idir)[ifreq]);
          Aphi_temp.push_back(expTemp);
        }
        c_AExpJphi.push_back(Aphi_temp);
      }

      InternalUpdate();
    }

    template<class OffshoreSystemType, class StretchingType, class WaveSpectrumType>
    void FrAiryIrregularOptimWaveField<OffshoreSystemType, StretchingType, WaveSpectrumType>::StepFinalize() {
      FrWaveField<OffshoreSystemType>::StepFinalize();
      InternalUpdate();
    }

    template<class OffshoreSystemType, class StretchingType, class WaveSpectrumType>
    void FrAiryIrregularOptimWaveField<OffshoreSystemType, StretchingType, WaveSpectrumType>::InternalUpdate() {
      c_expJwt.empty();
      c_expJwt.reserve(this->m_nbFreq);
      c_expJwt.clear();

      Complex expTemp;

      for (unsigned int ifreq = 0; ifreq < this->m_nbFreq; ++ifreq) {
        expTemp = exp(-JJ * this->m_waveFrequencies[ifreq] * this->c_time);
        c_expJwt.push_back(expTemp);
      }
    }

}  // end namespace frydom
