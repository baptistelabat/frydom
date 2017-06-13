//
// Created by frongere on 07/06/17.
//

#ifndef FRYDOM_FRTRYALFORCE_H
#define FRYDOM_FRTRYALFORCE_H

#include "../core/FrForce.h"

namespace frydom {


    class FrTryalForce : public FrForce {

      private:
        bool done;

      public:

        FrTryalForce();

        void UpdateState() override;

    };

}  //end namespace frydom

#endif //FRYDOM_FRTRYALFORCE_H
