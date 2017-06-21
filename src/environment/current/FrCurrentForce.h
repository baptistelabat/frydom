//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRCURRENTFORCE_H
#define FRYDOM_FRCURRENTFORCE_H

#include "../../core/FrForce.h"
#include "FrCurrent.h"
#include "../../core/FrHydroBody.h"

namespace frydom {

    class FrCurrentForce : public FrForce {

    public:

        void UpdateState();

    private:
        environment::FrCurrent* GetCurrent();

    };

}  // end namspace frydom

#endif //FRYDOM_FRCURRENTFORCE_H
