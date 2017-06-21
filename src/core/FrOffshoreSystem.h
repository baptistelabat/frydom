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
#include "../environment/waves/FrFreeSurface.h"
#include "../environment/current/FrCurrent.h"

namespace frydom {

    /// Abstract base class for a free surface model including wave modeling
//    class FrFreeSurface;  // forward declaration

    // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
    class FrOffshoreSystem :
            public chrono::ChSystemSMC,
            public std::enable_shared_from_this<FrOffshoreSystem> {

    private:
        double m_g_acc_magnitude;  ///< The local acceleration of gravity
        double m_water_density;

        std::unique_ptr<environment::FrFreeSurface> m_free_surface;  ///< The free surface's mesh that is a cartesian grid.
        std::unique_ptr<environment::FrCurrent> m_current;           ///< The current field model
        chrono::ChFrame<double> NEDframe;                            ///< Frame that has Z pointing down to have a well defined heading


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

        /// Add a current field to the system
        void setCurrent(environment::FrCurrent* current_field);

        /// Get the free surface model from the offshore system.
        environment::FrFreeSurface* getFreeSurface() const;

        /// get the current field model from the offshore system
        environment::FrCurrent* GetCurrent() const;

        /// Get/Set the value of the acceleration of gravity
        /// It must be given positive, in m/s**2
        void SetGravityAcceleration(double grav);
        double GetGravityAcceleration() const { return m_g_acc_magnitude; }

        /// Get/Set the water density
        double GetWaterDensity() { return m_water_density; }
        void SetWaterDensity(double rho) { m_water_density = rho; }

        /// Get NED frame
        chrono::ChFrame<double> GetNEDFrame() const { return NEDframe; }

    };

    // UTILITY FUNCTIONS

    /// Transform either a NED vector into a NWU vector or a NWU vector into a NED vector (inline)
    template <class Real=double>
    inline chrono::ChVector<Real> swap_NED_NWU(chrono::ChVector<Real> const vect) {
        auto new_vect = vect;
        new_vect.y() = - new_vect.y();
        new_vect.z() = - new_vect.z();
        return new_vect;
    }

    /// Transform a NED vector into NWU
    template <class Real=double>
    inline chrono::ChVector<Real> NED2NWU(chrono::ChVector<Real> const vect) {
        return swap_NED_NWU(vect);
    }

    /// Transform a NWU vector into NED
    template <class Real=double>
    inline chrono::ChVector<Real> NWU2NED(chrono::ChVector<Real> const vect) {
        return swap_NED_NWU(vect);
    }




} // end namespace frydom

/// Class to tranform vectors from world csys to NED csys
template <class Real>
chrono::ChVector<Real> TransformDirectionWorldToNED(const chrono::ChFrame<Real>& NEDFrame,
                                                    const chrono::ChVector<Real>& myvec) {
    return NEDFrame.GetA().MatrT_x_Vect(myvec);
}

/// Class to transform vectors from NED csys to world csys
template <class Real>
chrono::ChVector<Real> TransformDirectionNEDToWorld(const chrono::ChFrame<Real>& NEDFrame,
                                                    const chrono::ChVector<Real>& myvec) {
    return NEDFrame.GetA().Matr_x_Vect(myvec);
}



#endif //FRYDOM_FROFFSHORESYSTEM_H
