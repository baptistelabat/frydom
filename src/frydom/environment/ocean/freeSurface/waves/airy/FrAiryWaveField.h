//
// Created by frongere on 17/09/19.
//

#ifndef FRYDOM_FRAIRYWAVEFIELD_H
#define FRYDOM_FRAIRYWAVEFIELD_H

#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "FrKinematicStretching.h"

namespace frydom {

    class FrFreeSurface;


    template <class StretchingType>
    class FrAiryWaveField : public FrWaveField {

     protected:

      std::unique_ptr<StretchingType> m_verticalFactor;    ///< Vertical scale velocity factor with stretching

     public:

      explicit FrAiryWaveField(FrFreeSurface* freeSurface) : FrWaveField(freeSurface) {};


    };

}  // end namespace frydom



#endif //FRYDOM_FRAIRYWAVEFIELD_H
