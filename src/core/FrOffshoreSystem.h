//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

//#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChMatrixNM.h"
#include "chrono/core/ChMatrix33.h"
#include "../environment/waves/FrFreeSurface.h"

namespace frydom {

    /// Abstract base class for a free surface model including wave modeling
//    class FrFreeSurface;  // forward declaration

    // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
    class FrOffshoreSystem :
            public chrono::ChSystemSMC,
            public std::enable_shared_from_this<FrOffshoreSystem> {

    private:
        double m_g_acc_magnitude;  ///< The local acceleration of gravity

        std::unique_ptr<environment::FrFreeSurface> m_free_surface;  ///< The free surface's mesh that is a cartesian grid.
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

        /// Get the free surface model from the offshore system.
        environment::FrFreeSurface* getFreeSurface() const;

        /// Get/Set the value of the acceleration of gravity
        /// It must be given positive, in m/s**2
        void SetGravityAcceleration(double grav);
        double GetGravityAcceleration() const { return m_g_acc_magnitude; }

        /// Get NED frame
        chrono::ChFrame<double> GetNEDFrame() const { return NEDframe; }

    };

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
