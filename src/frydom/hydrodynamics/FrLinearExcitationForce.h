//
// Created by frongere on 30/10/17.
//

#ifndef FRYDOM_FRLINEAREXCITATIONFORCE_H
#define FRYDOM_FRLINEAREXCITATIONFORCE_H

#include <frydom/core/FrForce.h>
#include <frydom/environment/waves/FrWaveProbe.h>
#include "FrHydroDB.h"

namespace frydom {

    class FrLinearExcitationForce : public FrForce {

    private:

        std::shared_ptr<FrWaveProbe> m_waveProbe;

        std::vector<std::complex<double>> m_steadyForce;

    public:

        void SetWaveProbe(std::shared_ptr<FrWaveProbe> waveProbe) { m_waveProbe = waveProbe; }

        void Initialize() {
            // TODO: creer une steady force !!
        }

        void UpdateState() override {

            // Get the wave elevation
//            auto cmplxElevation = m_waveProbe->GetCmplxElevation();
//
//            return;

        }



    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
