//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem() {
//        auto system_ptr = std::make_shared(this);
        m_free_surface = std::make_unique<environment::FrFlatFreeSurface>(2);
    }

//    std::unique_ptr<environment::FrFreeSurface> FrOffshoreSystem::getFreeSurface() {
//        return m_free_surface;
//    }
}