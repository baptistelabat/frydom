//
// Created by Lucas Letournel on 07/01/19.
//

#ifndef FRYDOM_FRASSET_H
#define FRYDOM_FRASSET_H

#include <chrono/assets/ChAsset.h>
#include "frydom/core/FrBody.h"

namespace frydom {

    class FrAsset {

    protected:

        virtual std::shared_ptr<chrono::ChAsset> GetChronoAsset() = 0;

    public:

        friend void FrBody_::AddAsset(std::shared_ptr<FrAsset>);

    };

}   // end namespace frydom
#endif //FRYDOM_FRASSET_H
