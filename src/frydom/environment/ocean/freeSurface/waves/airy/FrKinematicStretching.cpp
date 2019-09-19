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

    void FrKinematicStretching::SetInfDepth(bool infinite_depth) { c_infinite_depth = infinite_depth; }

    void FrKinematicStretching::SetInfDepth_ON() { this->SetInfDepth(true); }

    void FrKinematicStretching::SetSteady(bool steady) { is_steady = steady; }

    bool FrKinematicStretching::IsSteady() const { return is_steady; }

    double FrKinematicStretching::Eval(const double &z, const double &konde, const double &depth) const {
        return Ez(z, konde, depth);
    }

    double FrKinematicStretching::Eval(const double &x, const double &y, const double &z, const double &konde,
                                       const double &depth) const {
        return Ez(z, konde, depth);
    }

    double FrKinematicStretching::Eval(const chrono::ChVector<> &pos, const double &konde, const double &depth) const {
        return Eval(pos.z(), konde, depth);
    }

    std::vector<double>
    FrKinematicStretching::Eval(const double &x, const double &y, const double &z, const std::vector<double> &vkonde,
                                const double &depth) const {
        std::vector<double> result;
        for (auto& konde: vkonde) {
            result.push_back( Eval(x, y, z, konde, depth) );
        }
        return result;
    }

    double FrKinematicStretching::EvalDZ(const double &z, const double &konde, const double &depth) const {
        return diffEz(z, konde, depth);
    }

    double FrKinematicStretching::EvalDZ(const double &x, const double &y, const double &z, const double &konde,
                                         const double &depth) const {
        return EvalDZ(z, konde, depth);
    }

    double FrKinematicStretching::EvalDZ(const chrono::ChVector<> &pos, const double &konde, const double &depth) const {
        return EvalDZ(pos.z(), konde, depth);
    }

    std::vector<double>
    FrKinematicStretching::EvalDZ(const double &x, const double &y, const double &z, const std::vector<double> &vkonde,
                                  const double &depth) const {
        std::vector<double> result;
        for (auto& konde: vkonde) {
            result.push_back( EvalDZ(x, y, z, konde, depth) );
        }
        return result;
    }

    double FrKinematicStretching::Ez(const double &z, const double &konde, const double &depth) const {
        if (c_infinite_depth) {
            return exp(konde * z);
        } else {
            return cosh(konde*(z+depth)) / sinh(konde*depth);
        }
    }

    double FrKinematicStretching::diffEz(const double &z, const double &konde, const double &depth) const {
        if (c_infinite_depth) {
            return konde * exp(konde * z);
        } else {
            return konde* sinh(konde*(z+depth)) / sinh(konde*depth);
        }
    }

    // --------------------------------------------------------
    // Vertical stretching
    // --------------------------------------------------------

    double FrKinStretchingVertical::Eval(const double &z, const double &konde, const double &depth) const {
        if (z < DBL_EPSILON) {
            return Ez(z, konde, depth);
        } else {
            return Ez(0., konde, depth);
        }
    }

    double FrKinStretchingVertical::EvalDZ(const double &z, const double &konde, const double &depth) const {
        if (z < DBL_EPSILON) {
            return diffEz(z, konde, depth);
        } else {
            return diffEz(0., konde, depth);
        }
    }

    // -------------------------------------------------------
    // Extrapolation stretching
    // -------------------------------------------------------

    double FrKinStretchingExtrapol::Eval(const double &z, const double &konde, const double &depth) const {
        if (z < DBL_EPSILON) {
            return Ez(z, konde, depth);
        } else {
            return Ez(0., konde, depth) + konde * z;
        }
    }

    double FrKinStretchingExtrapol::EvalDZ(const double &z, const double &konde, const double &depth) const {
        if (z < DBL_EPSILON) {
            return diffEz(z, konde, depth);
        } else {
            return diffEz(0., konde, depth) + konde;
        }
    }

    // --------------------------------------------------------
    // Wheeler stretching
    // --------------------------------------------------------

    double FrKinStretchingWheeler::Eval(const double &x, const double &y, const double &z,
                                        const double &konde, const double &depth) const {

        auto eta = m_waveField->GetElevation(x, y, NWU);
        auto rz = (depth + z) / (depth + eta);
        auto zp = depth * (rz - 1.);

        return Ez(zp, konde, depth);

    }


    double FrKinStretchingWheeler::EvalDZ(const double &x, const double &y, const double &z,
                                          const double &konde, const double &depth) const {

        auto eta = m_waveField->GetElevation(x, y, NWU);
        auto rz = (depth + z) / (depth + eta);
        auto drz = depth / (depth + eta);
        auto zp = depth * (rz - 1.);

        return diffEz(zp, konde, depth) * drz;

    }

    FrKinStretchingWheeler::FrKinStretchingWheeler(FrWaveField* waveField) : m_waveField(waveField) {
        SetSteady(false);
    }

//    void FrKinStretchingWheeler::SetWaveField(FrWaveField *waveField) { m_waveField = waveField; }

    double FrKinStretchingWheeler::Eval(const double &z, const double &konde, const double &depth) const {
        std::cout << "warning : 3D-coordinate is missing for wheeler stretching" << std::endl;
        return 1.;
    }

    double FrKinStretchingWheeler::EvalDZ(const double &z, const double &konde, const double &depth) const {
        std::cout << "warning : 3D-coordinate is missing for wheeler stretching" << std::endl;
        return 1.;
    }

    // -------------------------------------------------------------------
    // Chakrabati
    // -------------------------------------------------------------------

    double FrKinStretchingChakrabarti::Eval(const double &x, const double &y, const double &z,
                                            const double &konde, const double &depth) const {
        if (c_infinite_depth) {
            return exp(konde * z);
        } else {
            auto eta = m_waveField->GetElevation(x, y, NWU);
            return cosh(konde * (z + depth)) / sinh(konde * (depth + eta));
        }
    }

    double FrKinStretchingChakrabarti::EvalDZ(const double &x, const double &y, const double &z,
                                              const double &konde, const double &depth) const {

        if (c_infinite_depth) {
            return konde * exp(konde * z);
        } else {
            auto eta = m_waveField->GetElevation(x, y, NWU);
            return konde * sinh(konde * (z + depth)) / sinh(konde * (depth + eta));
        }

    }

    FrKinStretchingChakrabarti::FrKinStretchingChakrabarti(FrWaveField *waveField) : m_waveField(waveField) {
        SetSteady(false);
    }

//    void FrKinStretchingChakrabarti::SetWaveField(FrWaveField *waveField) { m_waveField = waveField; }

    // ---------------------------------------------------------------------
    // Delta-stretching
    // ---------------------------------------------------------------------

    double FrKinStretchingDelta::Eval(const double& x, const double& y, const double& z,
                                      const double& konde, const double& depth) const {

        auto zp = Zp(x, y, z);
        if (zp < DBL_EPSILON) {
            return Ez(zp, konde, depth);
        } else {
            return Ez(0., konde, depth) + konde * zp;
        }
    }

    double FrKinStretchingDelta::EvalDZ(const double& x, const double& y, const double& z,
                                        const double& konde, const double& depth) const {

        auto zp = Zp(x, y, z);
        auto drz = DZp(x, y, z);
        if (zp < DBL_EPSILON) {
            return diffEz(zp, konde, depth) * drz;
        } else {
            return diffEz(0., konde, depth) * drz + konde;
        }

    }

    double FrKinStretchingDelta::Zp(const double &x, const double &y, const double &z) const {

        auto eta = m_waveField->GetElevation(x, y, NWU);

        double zp;
        if (z > -m_hd) {
            zp = (z + m_hd) * (m_hd + m_delta * eta) / (m_hd + eta) - m_hd;
        } else {
            zp = z;
        }
        return zp;
    }

    double FrKinStretchingDelta::DZp(const double &x, const double &y, const double &z) const {

        auto eta = m_waveField->GetElevation(x, y, NWU);

        double drz;
        if (z > -m_hd) {
            drz = (m_hd + m_delta * eta) / (m_hd + eta);
        } else {
            drz = 1;
        }
        return drz;
    }

    FrKinStretchingDelta::FrKinStretchingDelta(FrWaveField *waveField) : m_waveField(waveField),
                                                                         m_delta(0.3), m_hd(0.)
    {
        SetSteady(false);
    }

//    void FrKinStretchingDelta::SetWaveField(FrWaveField *waveField) { m_waveField = waveField; }

    void FrKinStretchingDelta::SetParam(double hd, double delta) {
        m_hd = hd;
        m_delta = delta;
    }

    // ---------------------------------------------------------------------
    // HDelta-stretching
    // ---------------------------------------------------------------------

    double FrKinStretchingHDelta::Eval(const double& x, const double& y, const double& z,
                                       const double& konde, const double& depth) const {

        auto zp = Zp(x, y, z, depth);
        if (zp < DBL_EPSILON) {
            return Ez(zp, konde, depth);
        } else {
            return Ez(0., konde, depth) + konde * zp;
        }
    }

    double FrKinStretchingHDelta::EvalDZ(const double& x, const double& y, const double& z,
                                         const double& konde, const double& depth) const {

        auto zp = Zp(x, y, z, depth);
        auto drz = DZp(x, y, z, depth);
        if (zp < DBL_EPSILON) {
            return diffEz(zp, konde, depth) * drz;
        } else {
            return diffEz(0., konde, depth) * drz + konde;
        }

    }

    double FrKinStretchingHDelta::Zp(const double &x, const double &y, const double &z, const double& depth) const {

        auto eta = m_waveField->GetElevation(x, y, NWU);
        return (z + depth) * (depth + m_delta * eta) / (depth + eta) - depth;

    }

    double FrKinStretchingHDelta::DZp(const double &x, const double &y, const double &z, const double& depth) const {

        auto eta = m_waveField->GetElevation(x, y, NWU);
        return (depth + m_delta * eta) / (depth + eta);

    }

    FrKinStretchingHDelta::FrKinStretchingHDelta(FrWaveField *waveField) : m_waveField(waveField)
    {
        SetSteady(false);
    }


    void FrKinStretchingHDelta::SetDelta(double delta) {
        m_delta = delta;
    }

}  // end namespace frydom
