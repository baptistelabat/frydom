//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem() {

        // Creating a default flat free surface
        m_free_surface = std::make_unique<environment::FrFlatFreeSurface>(this, 2.);

    }

}  // end namespace frydom
