//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

//#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "../environment/waves/FrFreeSurface.h"

namespace frydom {

    /// Abstract base class for a free surface model including wave modeling
//    class FrFreeSurface;  // forward declaration

    // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
    class FrOffshoreSystem :
            public chrono::ChSystemSMC,
            public std::enable_shared_from_this<FrOffshoreSystem> {

    public:
        /// Default constructor
        FrOffshoreSystem(bool use_material_properties = true,
                         unsigned int max_objects = 16000,
                         double scene_size = 500);

        /// Copy constructor
        FrOffshoreSystem(const FrOffshoreSystem&) {};

        /// Default destructor
        ~FrOffshoreSystem() {std::cout << "OffshoreSystem deleted" << "\n";};

        /// Get a shared pointer from the system
        std::shared_ptr<FrOffshoreSystem> getPtr();

        /// Add a free surface model to the system
        void setFreeSurface(environment::FrFreeSurface* freeSurface);

        /// Get the free surface model from the offshore system.
        environment::FrFreeSurface* getFreeSurface();

        double m_g_acc_magnitude; // TODO: faire un set/get

    private:
        /// The free surface's mesh that is a cartesian grid.
        std::unique_ptr<environment::FrFreeSurface> m_free_surface;

        /// A shared pointer to this
//            std::shared_ptr<FrOffshoreSystem> m_ptr_to_this;
    };

} // end namespace frydom

#endif //FRYDOM_FROFFSHORESYSTEM_H
