//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem() {
        m_free_surface = std::make_shared<environment::FrFlatFreeSurface>(2);
    }

    std::shared_ptr<environment::FrFreeSurface> FrOffshoreSystem::getFreeSurface() {
        return m_free_surface;
    }
}