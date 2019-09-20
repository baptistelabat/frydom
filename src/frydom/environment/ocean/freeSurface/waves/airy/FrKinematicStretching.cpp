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


#include "FrKinematicStretching.h"
#include "FrAiryWaveField.h"

namespace frydom {

    template<typename OffshoreSystemType>
    FrKinematicStretching<OffshoreSystemType>::FrKinematicStretching(FrWaveField<OffshoreSystemType> *wave_field)
        : m_waveField(wave_field) {}

    template<typename OffshoreSystemType>
    void
    FrKinematicStretching<OffshoreSystemType>::SetInfDepth(bool infinite_depth) { c_infinite_depth = infinite_depth; }

    template<typename OffshoreSystemType>
    void FrKinematicStretching<OffshoreSystemType>::SetInfDepth_ON() { this->SetInfDepth(true); }

    template<typename OffshoreSystemType>
    void FrKinematicStretching<OffshoreSystemType>::SetSteady(bool steady) { is_steady = steady; }

    template<typename OffshoreSystemType>
    bool FrKinematicStretching<OffshoreSystemType>::IsSteady() const { return is_steady; }

    template<typename OffshoreSystemType>
    double
    FrKinematicStretching<OffshoreSystemType>::Eval(const double &z, const double &konde, const double &depth) const {
      return Ez(z, konde, depth);
    }

    template<typename OffshoreSystemType>
    double FrKinematicStretching<OffshoreSystemType>::Eval(const double &x, const double &y, const double &z,
                                                           const double &konde,
                                                           const double &depth) const {
      return Ez(z, konde, depth);
    }

    template<typename OffshoreSystemType>
    double FrKinematicStretching<OffshoreSystemType>::Eval(const chrono::ChVector<> &pos, const double &konde,
                                                           const double &depth) const {
      return Eval(pos.z(), konde, depth);
    }

    template<typename OffshoreSystemType>
    std::vector<double>
    FrKinematicStretching<OffshoreSystemType>::Eval(const double &x, const double &y, const double &z,
                                                    const std::vector<double> &vkonde,
                                                    const double &depth) const {
      std::vector<double> result;
      for (auto &konde: vkonde) {
        result.push_back(Eval(x, y, z, konde, depth));
      }
      return result;
    }

    template<typename OffshoreSystemType>
    double
    FrKinematicStretching<OffshoreSystemType>::EvalDZ(const double &z, const double &konde, const double &depth) const {
      return this->diffEz(z, konde, depth);
    }

    template<typename OffshoreSystemType>
    double FrKinematicStretching<OffshoreSystemType>::EvalDZ(const double &x, const double &y, const double &z,
                                                             const double &konde,
                                                             const double &depth) const {
      return EvalDZ(z, konde, depth);
    }

    template<typename OffshoreSystemType>
    double
    FrKinematicStretching<OffshoreSystemType>::EvalDZ(const chrono::ChVector<> &pos, const double &konde,
                                                      const double &depth) const {
      return EvalDZ(pos.z(), konde, depth);
    }

    template<typename OffshoreSystemType>
    std::vector<double>
    FrKinematicStretching<OffshoreSystemType>::EvalDZ(const double &x, const double &y, const double &z,
                                                      const std::vector<double> &vkonde,
                                                      const double &depth) const {
      std::vector<double> result;
      for (auto &konde: vkonde) {
        result.push_back(EvalDZ(x, y, z, konde, depth));
      }
      return result;
    }

    template<typename OffshoreSystemType>
    double
    FrKinematicStretching<OffshoreSystemType>::Ez(const double &z, const double &konde, const double &depth) const {
      if (c_infinite_depth) {
        return exp(konde * z);
      } else {
        return cosh(konde * (z + depth)) / sinh(konde * depth);
      }
    }

    template<typename OffshoreSystemType>
    double
    FrKinematicStretching<OffshoreSystemType>::diffEz(const double &z, const double &konde, const double &depth) const {
      if (c_infinite_depth) {
        return konde * exp(konde * z);
      } else {
        return konde * sinh(konde * (z + depth)) / sinh(konde * depth);
      }
    }

    // --------------------------------------------------------
    // Vertical stretching
    // --------------------------------------------------------
    template<typename OffshoreSystemType>
    double
    FrKinStretchingVertical<OffshoreSystemType>::Eval(const double &z, const double &konde, const double &depth) const {
      if (z < DBL_EPSILON) {
        return this->Ez(z, konde, depth);
      } else {
        return this->Ez(0., konde, depth);
      }
    }

    template<typename OffshoreSystemType>
    double FrKinStretchingVertical<OffshoreSystemType>::EvalDZ(const double &z, const double &konde,
                                                               const double &depth) const {
      if (z < DBL_EPSILON) {
        return this->diffEz(z, konde, depth);
      } else {
        return this->diffEz(0., konde, depth);
      }
    }

    // -------------------------------------------------------
    // Extrapolation stretching
    // -------------------------------------------------------
    template<typename OffshoreSystemType>
    double
    FrKinStretchingExtrapol<OffshoreSystemType>::Eval(const double &z, const double &konde, const double &depth) const {
      if (z < DBL_EPSILON) {
        return this->Ez(z, konde, depth);
      } else {
        return this->Ez(0., konde, depth) + konde * z;
      }
    }

    template<typename OffshoreSystemType>
    double FrKinStretchingExtrapol<OffshoreSystemType>::EvalDZ(const double &z, const double &konde,
                                                               const double &depth) const {
      if (z < DBL_EPSILON) {
        return this->diffEz(z, konde, depth);
      } else {
        return this->diffEz(0., konde, depth) + konde;
      }
    }

    // --------------------------------------------------------
    // Wheeler stretching
    // --------------------------------------------------------
    template<typename OffshoreSystemType>
    double FrKinStretchingWheeler<OffshoreSystemType>::Eval(const double &x, const double &y, const double &z,
                                                            const double &konde, const double &depth) const {

      auto eta = this->m_waveField->GetElevation(x, y, NWU);
      auto rz = (depth + z) / (depth + eta);
      auto zp = depth * (rz - 1.);

      return Ez(zp, konde, depth);

    }

    template<typename OffshoreSystemType>
    double FrKinStretchingWheeler<OffshoreSystemType>::EvalDZ(const double &x, const double &y, const double &z,
                                                              const double &konde, const double &depth) const {

      auto eta = this->m_waveField->GetElevation(x, y, NWU);
      auto rz = (depth + z) / (depth + eta);
      auto drz = depth / (depth + eta);
      auto zp = depth * (rz - 1.);

      return diffEz(zp, konde, depth) * drz;

    }

    template<typename OffshoreSystemType>
    FrKinStretchingWheeler<OffshoreSystemType>::FrKinStretchingWheeler(FrWaveField<OffshoreSystemType> *waveField)
        : FrKinematicStretching<OffshoreSystemType>(waveField) {
      this->SetSteady(false);
    }

    template<typename OffshoreSystemType>
    double
    FrKinStretchingWheeler<OffshoreSystemType>::Eval(const double &z, const double &konde, const double &depth) const {
      std::cout << "warning : 3D-coordinate is missing for wheeler stretching" << std::endl;
      return 1.;
    }

    template<typename OffshoreSystemType>
    double FrKinStretchingWheeler<OffshoreSystemType>::EvalDZ(const double &z, const double &konde,
                                                              const double &depth) const {
      std::cout << "warning : 3D-coordinate is missing for wheeler stretching" << std::endl;
      return 1.;
    }

    // -------------------------------------------------------------------
    // Chakrabati
    // -------------------------------------------------------------------
    template<typename OffshoreSystemType>
    double FrKinStretchingChakrabarti<OffshoreSystemType>::Eval(const double &x, const double &y, const double &z,
                                                                const double &konde, const double &depth) const {
      if (this->c_infinite_depth) {
        return exp(konde * z);
      } else {
        auto eta = this->m_waveField->GetElevation(x, y, NWU);
        return cosh(konde * (z + depth)) / sinh(konde * (depth + eta));
      }
    }

    template<typename OffshoreSystemType>
    double FrKinStretchingChakrabarti<OffshoreSystemType>::EvalDZ(const double &x, const double &y, const double &z,
                                                                  const double &konde, const double &depth) const {

      if (this->c_infinite_depth) {
        return konde * exp(konde * z);
      } else {
        auto eta = this->m_waveField->GetElevation(x, y, NWU);
        return konde * sinh(konde * (z + depth)) / sinh(konde * (depth + eta));
      }

    }

    template<typename OffshoreSystemType>
    FrKinStretchingChakrabarti<OffshoreSystemType>::FrKinStretchingChakrabarti(
        FrWaveField<OffshoreSystemType> *waveField) : FrKinematicStretching<OffshoreSystemType>(waveField) {
      this->SetSteady(false);
    }

    // ---------------------------------------------------------------------
    // Delta-stretching
    // ---------------------------------------------------------------------
    template<typename OffshoreSystemType>
    double FrKinStretchingDelta<OffshoreSystemType>::Eval(const double &x, const double &y, const double &z,
                                                          const double &konde, const double &depth) const {

      auto zp = Zp(x, y, z);
      if (zp < DBL_EPSILON) {
        return this->Ez(zp, konde, depth);
      } else {
        return this->Ez(0., konde, depth) + konde * zp;
      }
    }

    template<typename OffshoreSystemType>
    double FrKinStretchingDelta<OffshoreSystemType>::EvalDZ(const double &x, const double &y, const double &z,
                                                            const double &konde, const double &depth) const {

      auto zp = Zp(x, y, z);
      auto drz = DZp(x, y, z);
      if (zp < DBL_EPSILON) {
        return this->diffEz(zp, konde, depth) * drz;
      } else {
        return this->diffEz(0., konde, depth) * drz + konde;
      }

    }

    template<typename OffshoreSystemType>
    double FrKinStretchingDelta<OffshoreSystemType>::Zp(const double &x, const double &y, const double &z) const {

      auto eta = this->m_waveField->GetElevation(x, y, NWU);

      double zp;
      if (z > -m_hd) {
        zp = (z + m_hd) * (m_hd + m_delta * eta) / (m_hd + eta) - m_hd;
      } else {
        zp = z;
      }
      return zp;
    }

    template<typename OffshoreSystemType>
    double FrKinStretchingDelta<OffshoreSystemType>::DZp(const double &x, const double &y, const double &z) const {

      auto eta = this->m_waveField->GetElevation(x, y, NWU);

      double drz;
      if (z > -m_hd) {
        drz = (m_hd + m_delta * eta) / (m_hd + eta);
      } else {
        drz = 1;
      }
      return drz;
    }

    template<typename OffshoreSystemType>
    FrKinStretchingDelta<OffshoreSystemType>::FrKinStretchingDelta(FrWaveField<OffshoreSystemType> *waveField)
        : FrKinematicStretching<OffshoreSystemType>(waveField),
          m_delta(0.3), m_hd(0.) {
      this->SetSteady(false);
    }

    template<typename OffshoreSystemType>
    void FrKinStretchingDelta<OffshoreSystemType>::SetParam(double hd, double delta) {
      m_hd = hd;
      m_delta = delta;
    }

    // ---------------------------------------------------------------------
    // HDelta-stretching
    // ---------------------------------------------------------------------
    template<typename OffshoreSystemType>
    double FrKinStretchingHDelta<OffshoreSystemType>::Eval(const double &x, const double &y, const double &z,
                                                           const double &konde, const double &depth) const {

      auto zp = Zp(x, y, z, depth);
      if (zp < DBL_EPSILON) {
        return this->Ez(zp, konde, depth);
      } else {
        return this->Ez(0., konde, depth) + konde * zp;
      }
    }

    template<typename OffshoreSystemType>
    double FrKinStretchingHDelta<OffshoreSystemType>::EvalDZ(const double &x, const double &y, const double &z,
                                                             const double &konde, const double &depth) const {

      auto zp = Zp(x, y, z, depth);
      auto drz = DZp(x, y, z, depth);
      if (zp < DBL_EPSILON) {
        return this->diffEz(zp, konde, depth) * drz;
      } else {
        return this->diffEz(0., konde, depth) * drz + konde;
      }

    }

    template<typename OffshoreSystemType>
    double FrKinStretchingHDelta<OffshoreSystemType>::Zp(const double &x, const double &y, const double &z,
                                                         const double &depth) const {

      auto eta = this->m_waveField->GetElevation(x, y, NWU);
      return (z + depth) * (depth + m_delta * eta) / (depth + eta) - depth;

    }

    template<typename OffshoreSystemType>
    double FrKinStretchingHDelta<OffshoreSystemType>::DZp(const double &x, const double &y, const double &z,
                                                          const double &depth) const {

      auto eta = this->m_waveField->GetElevation(x, y, NWU);
      return (depth + m_delta * eta) / (depth + eta);

    }

    template<typename OffshoreSystemType>
    FrKinStretchingHDelta<OffshoreSystemType>::FrKinStretchingHDelta(FrWaveField<OffshoreSystemType> *waveField)
        : FrKinematicStretching<OffshoreSystemType>(waveField) {
      this->SetSteady(false);
    }

    template<typename OffshoreSystemType>
    void FrKinStretchingHDelta<OffshoreSystemType>::SetDelta(double delta) {
      m_delta = delta;
    }

}  // end namespace frydom
