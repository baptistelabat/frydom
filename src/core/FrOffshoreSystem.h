//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

#include "chrono/physics/ChSystemNSC.h"
#include "../environment/waves/FrFreeSurface.h"

namespace frydom {

    // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
    class FrOffshoreSystem : public chrono::ChSystemNSC {
      public:
        /// Default constructor
        FrOffshoreSystem();

        /// Default destructor
        ~FrOffshoreSystem() {};

        /// Get the wave field model from the offshore system.
        std::shared_ptr<environment::FrFreeSurface> getFreeSurface();

      private:
        /// The free surface's mesh that is a cartesian grid.
        std::shared_ptr<environment::FrFreeSurface> m_free_surface;

    };

} // end namespace frydom

#endif //FRYDOM_FROFFSHORESYSTEM_H
