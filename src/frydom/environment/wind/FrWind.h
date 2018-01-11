//
// Created by frongere on 03/07/17.
//

#ifndef FRYDOM_FRWIND_H
#define FRYDOM_FRWIND_H

#include "frydom/core/FrObject.h"

namespace frydom {

    class FrWind : public FrObject {


    public:
        void Update(double time) {
            // TODO
        }

        virtual void Initialize() override {}

        virtual void StepFinalize() override {}

    };

}  // end namespace frydom
#endif //FRYDOM_FRWIND_H
