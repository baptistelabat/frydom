//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

//#include "chrono/physics/ChSystemNSC.h"
//#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChMatrixNM.h"
#include "chrono/core/ChMatrix33.h"
#include "frydom/environment/waves/FrFreeSurface.h"
#include "frydom/environment/current/FrCurrent.h"
#include "frydom/core/FrBody.h"

// TODO: les objets environnement devront etre mis dans une classe environnement qui encapsule tout l'environnement:
// vent, vagues, courant, fond...


namespace frydom {

//    namespace environment {
//        class FrCurrent;
//    }

    /// Abstract base class for a free surface model including wave modeling
//    class FrFreeSurface;  // forward declaration

    // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
    class FrOffshoreSystem :
            public chrono::ChSystemSMC,
            public std::enable_shared_from_this<FrOffshoreSystem> {  // TODO: supprimer cette dependance !

    private:
        double m_gravity_acc_magnitude = 9.81;  ///< The local acceleration of gravity
        double m_water_density = 1026.;
        double m_air_density = 1.204;

        std::shared_ptr<FrBody> world_body;

        std::unique_ptr<environment::FrFreeSurface> m_free_surface;  ///< The free surface's mesh that is a cartesian grid.
        std::unique_ptr<environment::FrCurrent> m_current;           ///< The current field model
        chrono::ChFrame<double> NEDframe;                            ///< Frame that has Z pointing down to have a well defined heading


    public:
        /// Default constructor
        explicit FrOffshoreSystem(bool use_material_properties = true,
                                  unsigned int max_objects = 16000,
                                  double scene_size = 500);

        /// Copy constructor
//        FrOffshoreSystem(const FrOffshoreSystem& system) {};

        /// Default destructor
//        ~FrOffshoreSystem() override {std::cout << "OffshoreSystem deleted" << "\n";};

        /// Add a free surface model to the system
        void setFreeSurface(environment::FrFreeSurface* freeSurface);

        /// Add a current field to the system
        void SetCurrent(environment::FrCurrent *current_field);

        /// Get the free surface model from the offshore system.
        environment::FrFreeSurface* getFreeSurface() const;

        /// get the current field model from the offshore system
        environment::FrCurrent* GetCurrent() const;

        /// Get/Set the value of the acceleration of gravity
        /// It must be given positive, in m/s**2
        void SetGravityAcceleration(double grav);
        double GetGravityAcceleration() const { return m_gravity_acc_magnitude; }

        /// Get the water density
        double GetWaterDensity() const { return m_water_density; }

        /// Set the water density
        void SetWaterDensity(const double rho_water) { m_water_density = rho_water; }

        ///Get the air density
        double GetAirDensity() const { return m_air_density; }

        /// Set the air density
        void SetAirDensity(const double rho_air) { m_air_density = rho_air; }

        /// Get NED frame
        chrono::ChFrame<double> GetNEDFrame() const { return NEDframe; }

        /// Get the world body
        chrono::ChBody* GetWorldBodyPtr() const {
            return world_body.get();
        }

        std::shared_ptr<chrono::ChBody> GetWorldBody() const {
            return world_body;
        }



        /// Updates all the auxiliary data and children of
        /// bodies, forces, links, given their current state
        /// as well as environment prior to everything.
        virtual void Update(bool update_assets = true) override;




    };  // class FrOffshoreSystem

} // end namespace frydom

#endif //FRYDOM_FROFFSHORESYSTEM_H
