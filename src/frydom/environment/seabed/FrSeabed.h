//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRSEABED_H
#define FRYDOM_FRSEABED_H

#include "frydom/core/FrObject.h"

namespace frydom {

    class FrSeabed  : public FrObject {

    public:

        void Update(double time) {}

        virtual void Initialize() override {}

        virtual void StepFinalize() override {}

    };

}  // end namespace frydom


#endif //FRYDOM_FRSEABED_H
