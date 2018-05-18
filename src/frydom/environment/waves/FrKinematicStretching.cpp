//
// Created by camille on 26/04/18.
//

#include "FrKinematicStretching.h"
#include "frydom/environment/waves/FrWaveField.h"

namespace frydom {

    // --------------------------------------------------------
    // Wheeler stretching
    // --------------------------------------------------------

    double FrKinStretchingWheeler::Eval(const double &x, const double &y, const double &z,
                                        const double &konde, const double &depth) const {

        auto eta = m_waveField->GetElevation(x, y);
        auto rz = (depth + z) / (depth + eta);
        auto zp = depth * (rz - 1.);

        return Ez(zp, konde, depth);

    }


    double FrKinStretchingWheeler::EvalDZ(const double &x, const double &y, const double &z,
                                          const double &konde, const double &depth) const {

        auto eta = m_waveField->GetElevation(x, y);
        auto rz = (depth + z) / (depth + eta);
        auto drz = depth / (depth + eta);
        auto zp = depth * (rz - 1.);

        return diffEz(zp, konde, depth) * drz;

    }

    // -------------------------------------------------------------------
    // Chakrabati
    // -------------------------------------------------------------------

    double FrKinStretchingChakrabarti::Eval(const double &x, const double &y, const double &z,
                                            const double &konde, const double &depth) const {
        if (m_infinite_depth) {
            return exp(konde * z);
        } else {
            auto eta = m_waveField->GetElevation(x, y);
            return cosh(konde * (z + depth)) / sinh(konde * (depth + eta));
        }
    }

    double FrKinStretchingChakrabarti::EvalDZ(const double &x, const double &y, const double &z,
                                              const double &konde, const double &depth) const {

        if (m_infinite_depth) {
            return konde * exp(konde * z);
        } else {
            auto eta = m_waveField->GetElevation(x, y);
            return konde * sinh(konde * (z + depth)) / sinh(konde * (depth + eta));
        }

    }

    // ---------------------------------------------------------------------
    // Delta-stretching
    // ---------------------------------------------------------------------

    double FrKinStretchingDelta::Eval(const double& x, const double& y, const double& z,
                                      const double& konde, const double& depth) const {

        auto zp = Zp(x, y, z);
        return Ez(zp, konde, depth);
    }

    double FrKinStretchingDelta::EvalDZ(const double& x, const double& y, const double& z,
                                        const double& konde, const double& depth) const {

        auto zp = Zp(x, y, z);
        return diffEz(zp, konde, depth);

    }

    double FrKinStretchingDelta::Zp(const double &x, const double &y, const double &z) const {

        auto eta = m_waveField->GetElevation(x, y);

        double zp;
        if (z > -m_hd) {
            zp = (z + m_hd) * (m_hd + m_delta * eta) / (m_hd + eta) - m_hd;
        } else {
            zp = z;
        }
    }

}