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
    class FrOffshoreSystem : public chrono::ChSystemNSC, public std::enable_shared_from_this<FrOffshoreSystem> {

    public:
        /// Default constructor
        FrOffshoreSystem(unsigned int max_objects = 16000,
                         double scene_size = 500,
                         bool init_sys = true);

        /// Copy constructor
        FrOffshoreSystem(const FrOffshoreSystem&) {
//            FrOffshoreSystem* system = this;
        };

        /// Default destructor
        ~FrOffshoreSystem() {};

        /// Get a shared pointer from the system
        std::shared_ptr<FrOffshoreSystem> getPtr();

        /// Add a free surface model to the system
//        template<typename FS_type>
        void setFreeSurface(environment::FrFreeSurface* freeSurface);

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
