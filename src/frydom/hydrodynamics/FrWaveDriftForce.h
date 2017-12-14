//
// Created by camille on 14/12/17.
//

#ifndef FRYDOM_FRWAVEDRIFTFORCE_H
#define FRYDOM_FRWAVEDRIFTFORCE_H

#include <frydom/core/FrForce.h>
#include <frydom/environment/waves/FrWaveProbe.h>

namespace frydom {

    class FrWaveDriftForce : public FrForce {

    public:

        /// Wave probe attached to the force at the application point location
        void SetWaveProbe(std::shared_ptr<FrLinearWaveProbe>& waveProbe) { m_waveProbe = waveProbe; }

        /// Update procedure containing the Wave Drift Force definition
        void UpdateState() override;

    private:

        std::shared_ptr<FrLinearWaveProbe> m_waveProbe;    ///< Wave probe for local wave field characteristics


    };

// end namespace frydom
}

#endif //FRYDOM_FRWAVEDRIFTFORCE_H
