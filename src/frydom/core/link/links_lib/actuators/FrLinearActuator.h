//
// Created by frongere on 06/02/19.
//

#ifndef FRYDOM_FRLINEARACTUATOR_H
#define FRYDOM_FRLINEARACTUATOR_H

#include "FrActuator.h"

namespace frydom {

    // Forward declaration
    class FrLink;

    class FrLinearActuator : public FrActuator {

    public:
        explicit FrLinearActuator(FrLink *actuatedLink);


            // TODO : ajouter des methodes communes a tous les actuateurs lineaires

    };

} //end namespace frydom

#endif //FRYDOM_FRLINEARACTUATOR_H
