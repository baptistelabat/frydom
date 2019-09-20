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


#include "FrAiryRegularOptimWaveField.h"

namespace frydom {

    template<class OffshoreSystemType, class StretchingType>
    void FrAiryRegularOptimWaveField<OffshoreSystemType, StretchingType>::InternalUpdate() {
      c_expJwt = this->m_height * std::exp(-JJ * this->m_omega * this->c_time); // m_height = Amplitude.
      c_cosTheta = std::cos(this->m_dirAngle);
      c_sinTheta = std::sin(this->m_dirAngle);
    }

    template<class OffshoreSystemType, class StretchingType>
    void FrAiryRegularOptimWaveField<OffshoreSystemType, StretchingType>::Initialize() {
      FrWaveField<OffshoreSystemType>::Initialize();
      InternalUpdate();
    }

    template<class OffshoreSystemType, class StretchingType>
    void FrAiryRegularOptimWaveField<OffshoreSystemType, StretchingType>::StepFinalize() {
      FrWaveField<OffshoreSystemType>::StepFinalize();
      InternalUpdate();
    }

    template<class OffshoreSystemType, class StretchingType>
    std::vector<std::vector<Complex>>
    frydom::FrAiryRegularOptimWaveField<OffshoreSystemType, StretchingType>::GetComplexElevation(double x, double y,
                                                                             FRAME_CONVENTION fc) const {
      double NWUsign = 1;
      if (IsNED(fc)) {
        y = -y;
        NWUsign = -NWUsign;
      }
      double kdir = x * c_cosTheta + y * c_sinTheta;
      Complex cmplxElevation =
          c_expJwt * std::exp(JJ * this->m_k * kdir) * NWUsign * this->c_ramp; // m_height is included in c_expJwt.
      return std::vector<std::vector<Complex>>(1, std::vector<Complex>(1, cmplxElevation));
    }

    template<class OffshoreSystemType, class StretchingType>
    mathutils::Vector3d<frydom::Complex>
    FrAiryRegularOptimWaveField<OffshoreSystemType, StretchingType>::GetComplexVelocity(double x, double y, double z,
                                                                    FRAME_CONVENTION fc) const {
      double NWUsign = 1;
      if (IsNED(fc)) {
        y = -y;
        z = -z;
        NWUsign = -NWUsign;
      }
      auto ComplexElevation = GetComplexElevation(x, y, fc);

      auto Vtemp =
          this->m_omega * ComplexElevation[0][0] * this->m_verticalFactor->Eval(x, y, z, this->m_k, this->c_depth);

      auto Vx = c_cosTheta * Vtemp * NWUsign;
      auto Vy = c_sinTheta * Vtemp;
      auto Vz = -JJ * this->m_omega / this->m_k * ComplexElevation[0][0] *
                this->m_verticalFactor->EvalDZ(x, y, z, this->m_k, this->c_depth);

      return {Vx, Vy, Vz};
    }

    template<class OffshoreSystemType, class StretchingType>
    FrAiryRegularOptimWaveField<OffshoreSystemType, StretchingType>::FrAiryRegularOptimWaveField(frydom::FrFreeSurface<OffshoreSystemType> *freeSurface)
        : FrAiryRegularWaveField<OffshoreSystemType, StretchingType>(freeSurface) {}

}  // end namespace frydom
