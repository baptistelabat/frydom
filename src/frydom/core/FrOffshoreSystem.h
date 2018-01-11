//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

//#include "chrono/physics/ChSystemNSC.h"
//#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/core/ChMatrixNM.h"
#include "chrono/core/ChMatrix33.h"

#include "FrObject.h"
#include "frydom/environment/waves/FrFreeSurface.h"
#include "frydom/environment/current/FrCurrent.h"
#include "frydom/core/FrBody.h"

#include "frydom/environment/FrEnvironment.h"

// TODO: les objets environnement devront etre mis dans une classe environnement qui encapsule tout l'environnement:
// vent, vagues, courant, fond...


namespace frydom {

//    namespace environment {
//        class FrCurrent;
//    }

    /// Abstract base class for a free surface model including wave modeling
//    class FrFreeSurface;  // forward declaration

//    class FrEnvironment;

    // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
    class FrOffshoreSystem :
            public chrono::ChSystemSMC,
            public std::enable_shared_from_this<FrOffshoreSystem>,
            public FrObject
    {  // TODO: supprimer cette dependance !

    private:
//        double m_gravity_acc_magnitude = 9.81;  ///< The local acceleration of gravity
//        double m_water_density = 1026.;
//        double m_air_density = 1.204;

        std::shared_ptr<FrBody> world_body;

//        std::unique_ptr<FrFreeSurface> m_free_surface;  ///< The free surface's mesh that is a cartesian grid.
//        std::unique_ptr<FrCurrent> m_current;           ///< The current field model
        chrono::ChFrame<double> NEDframe;                            ///< Frame that has Z pointing down to have a well defined heading

        std::unique_ptr<FrEnvironment> m_environment;

    public:
        /// Default constructor
        explicit FrOffshoreSystem(bool use_material_properties = true,
                                  unsigned int max_objects = 16000,
                                  double scene_size = 500);

        /// Copy constructor
//        FrOffshoreSystem(const FrOffshoreSystem& system) {};

        /// Default destructor
//        ~FrOffshoreSystem() override {std::cout << "OffshoreSystem deleted" << "\n";};

        inline FrEnvironment* GetEnvironment() const {
            return m_environment.get();
        }

        inline FrFreeSurface* GetFreeSurface() const {
            return m_environment->GetFreeSurface();
        }

        inline FrCurrent* GetCurrent() const {
            return m_environment->GetCurrent();
        }

//        inline FrTidal* GetTidal() const {  // TODO:remettre
//            return m_environment->GetTidal();
//        }

        inline FrSeabed* GetSeabed() const {
            return m_environment->GetSeabed();
        }

        void SetFreeSurfaceGrid(double lmin, double lmax, double dl) {
            m_environment->GetFreeSurface()->SetGrid(lmin, lmax, dl);
        }

        void SetFreeSurfaceGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {
            m_environment->GetFreeSurface()->SetGrid(xmin, xmax, dx, ymin, ymax, dy);
        }

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

        virtual void StateScatter(const chrono::ChState& x, const chrono::ChStateDelta& v, const double T) override;

        virtual bool Integrate_Y() override;

        virtual void CustomEndOfStep() override;

        virtual void Initialize() override;


    };  // class FrOffshoreSystem

} // end namespace frydom

#endif //FRYDOM_FROFFSHORESYSTEM_H
