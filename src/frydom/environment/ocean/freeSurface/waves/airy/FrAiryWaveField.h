//
// Created by frongere on 17/09/19.
//

#ifndef FRYDOM_FRAIRYWAVEFIELD_H
#define FRYDOM_FRAIRYWAVEFIELD_H

#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "FrKinematicStretching.h"

namespace frydom {

    template<typename OffshoreSystemType>
    class FrFreeSurface;


    template<class OffshoreSystemType, class StretchingType>
    class FrAiryWaveField : public FrWaveField<OffshoreSystemType> {

     protected:

      std::unique_ptr<StretchingType> m_verticalFactor;    ///< Vertical scale velocity factor with stretching

     public:

      explicit FrAiryWaveField(FrFreeSurface<OffshoreSystemType> *freeSurface) : FrWaveField<OffshoreSystemType>(
          freeSurface) {};


    };

}  // end namespace frydom



#endif //FRYDOM_FRAIRYWAVEFIELD_H
