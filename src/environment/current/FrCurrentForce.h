//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRCURRENTFORCE_H
#define FRYDOM_FRCURRENTFORCE_H

#include "core/FrForce.h"
#include "FrCurrent.h"

namespace frydom {
namespace environment {

    class FrCurrentForce : public FrForce {

    public:

        void UpdateState();

        void SetPolarCoeffTable() {}; // TODO: mettre cela dans une classe derivee


    private:

        environment::FrCurrent *GetCurrent();

    };
}  // end namespace environment
}  // end namespace frydom

#endif //FRYDOM_FRCURRENTFORCE_H
