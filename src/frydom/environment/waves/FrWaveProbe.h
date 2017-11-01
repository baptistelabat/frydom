//
// Created by frongere on 30/10/17.
//

#ifndef FRYDOM_FRWAVEPROBE_H
#define FRYDOM_FRWAVEPROBE_H

#include <memory>
#include <complex>

#include "chrono/core/ChVector.h"
#include "FrWaveField.h"

namespace frydom {

    // Forward declaration
    class FrWaveField;

    /// Class to make a measurement of the wave elevation with respect to the mean water height (tidal)
    /// It is mainly used at a fixed absolute position in the local horizontal plane
    class FrWaveProbe {
    protected:

        double m_x = 0;
        double m_y = 0;

        FrWaveField* m_waveField;

    };

    class FrLinearWaveField;

    class FrLinearWaveProbe : public FrWaveProbe {

    private:
        FrLinearWaveField* m_waveField;

    public:


    };










}  // end namespace frydom


#endif //FRYDOM_FRWAVEPROBE_H

