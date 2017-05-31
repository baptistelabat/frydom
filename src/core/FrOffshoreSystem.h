//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

#include "chrono/physics/ChSystemNSC.h"
#include "../environment/waves/FrFreeSurface.h"

namespace frydom {

        class FrFreeSurface;  // forward declaration

        // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
        class FrOffshoreSystem : public chrono::ChSystemNSC {

        public:
            /// Default constructor
            FrOffshoreSystem();

            /// Default destructor
            ~FrOffshoreSystem() {};

            /// Add a free surface model to the system
            void setFreeSurface(std::unique_ptr<environment::FrFreeSurface>) {};

            /// Get the free surface model from the offshore system.
            std::unique_ptr<environment::FrFreeSurface> getFreeSurface() { return 0; };

        private:
            /// The free surface's mesh that is a cartesian grid.
            std::unique_ptr<environment::FrFreeSurface> m_free_surface;

            /// A shared pointer to this
//            std::shared_ptr<FrOffshoreSystem> m_ptr_to_this;
        };

} // end namespace frydom

#endif //FRYDOM_FROFFSHORESYSTEM_H
