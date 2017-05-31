//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem() {}

    std::shared_ptr<FrOffshoreSystem> FrOffshoreSystem::getPtr() {
        return shared_from_this();
    }

}  // end namespace frydom
