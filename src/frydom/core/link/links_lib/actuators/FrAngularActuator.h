//
// Created by frongere on 06/02/19.
//

#ifndef FRYDOM_FRANGULARACTUATOR_H
#define FRYDOM_FRANGULARACTUATOR_H


#include "FrActuator.h"


namespace frydom {

    // Forward declaration
    class FrLink_;

    class FrAngularActuator : public FrActuator {

    public:
        FrAngularActuator(FrLink_* actuatedLink);


        // TODO : ajouter des methodes communes a tous les actuateurs angulaires tel que GetRPM()...


    };

}  // end namespace frydom

#endif //FRYDOM_FRANGULARACTUATOR_H
