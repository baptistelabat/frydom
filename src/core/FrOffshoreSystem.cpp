//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem(unsigned int max_objects, double scene_size, bool init_sys) :
            chrono::ChSystemNSC(max_objects, scene_size, init_sys) {

//        m_free_surface = environment::FrFreeSurface
    }

    std::shared_ptr<FrOffshoreSystem> FrOffshoreSystem::getPtr() {
        return shared_from_this();
    }

//    template<>
    void FrOffshoreSystem::setFreeSurface(environment::FrFreeSurface& freeSurface) {

//        m_free_surface = std::make_unique<environment::FrFreeSurface>(freeSurface);

    }

}  // end namespace frydom
