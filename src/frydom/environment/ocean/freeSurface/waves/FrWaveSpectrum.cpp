//
// Created by frongere on 04/10/17.
//

#include "FrWaveSpectrum.h"

namespace frydom {

    std::unique_ptr<FrWaveSpectrum> MakeWaveSpectrum(WAVE_SPECTRUM_TYPE type) {

        switch (type) {

            case JONSWAP:
                return std::make_unique<FrJonswapWaveSpectrum>();

            case PIERSON_MOSKOWITZ:
                return std::make_unique<FrPiersonMoskowitzWaveSpectrum>();

        }

    }
}  // end namespace frydom