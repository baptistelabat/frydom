//
// Created by camille on 14/12/17.
//

#ifndef FRYDOM_FRWAVEDRIFTFORCE_H
#define FRYDOM_FRWAVEDRIFTFORCE_H

#include <frydom/core/FrForce.h>
#include <frydom/environment/waves/FrWaveProbe.h>
#include <frydom/core/FrHydroBody.h>

namespace frydom {

    class FrWaveDriftForce : public FrForce {

    public:

        /// Construct a new force model from drift table coefficients
        FrWaveDriftForce(const std::string hdf5_file);

        /// Wave probe attached to the force at the application point location
        void SetWaveProbe(std::shared_ptr<FrLinearWaveProbe>& waveProbe) { m_waveProbe = waveProbe; }

        /// Definition of the complex amplitude at the wave probe
        void SetCmplxElevation();

        /// Definition of the body where the force is applied
        void SetBody(FrHydroBody* body) { m_body = body;}

        /// Update procedure containing the Wave Drift Force definition
        void UpdateState() override;

    private:

        std::shared_ptr<FrLinearWaveProbe> m_waveProbe;    ///< Wave probe for local wave field characteristics
        std::vector<std::vector<std::complex<double>>> m_CmplxElevation;  ///< Wave complex elevation
        LookupTable2d<> m_table;                           ///< Table of the wave drift force depending on freq. and dir.
        FrHydroBody*    m_body;                            ///< Hydro body to which the force is applied

    };

// end namespace frydom
}

#endif //FRYDOM_FRWAVEDRIFTFORCE_H
