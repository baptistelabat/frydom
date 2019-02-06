//
// Created by frongere on 06/02/19.
//

#ifndef FRYDOM_FRACTUATOR_H
#define FRYDOM_FRACTUATOR_H

#include <memory>

#include "frydom/core/link/FrLinkBase.h"

namespace frydom {

    // Forward declaration
    class FrLink_;

    class FrActuator : public FrLinkBase_ {

    private:
        FrLink_* m_associatedLink;

    public:
        FrActuator(FrLink_* associatedLink);


        // TODO : ajouter des methodes communes a tous les actuateurs tel que GetPower() ...





    };


}  // end namespace frydom


#endif //FRYDOM_FRACTUATOR_H
