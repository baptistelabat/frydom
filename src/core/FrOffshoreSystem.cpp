//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem() {

        // Creating the first pointer to this
//        auto ptr = std::make_shared<FrOffshoreSystem>(this);
//        shared_from_this();


        // Creating a default flat free surface

        environment::FrFlatFreeSurface freeSurface(this, 2.);
//        m_free_surface = environment::FrFlatFreeSurface(this, 2.);
//        environment::FrFlatFreeSurface* fs_ptr = &freeSurface;
        environment::FrFlatFreeSurface* fs_ptr = &freeSurface;
        m_free_surface.reset(fs_ptr);


    }

}  // end namespace frydom
