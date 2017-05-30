//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem() {

        // Creating a default flat free surface that may be changed
        environment::FrFlatFreeSurface freeSurface(this, 2.);
        m_free_surface.reset(&freeSurface);

    }

}  // end namespace frydom
