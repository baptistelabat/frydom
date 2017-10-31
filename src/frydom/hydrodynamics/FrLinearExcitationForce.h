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

    public:

        void SetWaveProbe(std::shared_ptr<FrWaveProbe> waveProbe) { m_waveProbe = waveProbe; }










        void UpdateState() override {
            // TODO
        }



    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
