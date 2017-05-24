// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// Base class for a water free surface system
//
// =============================================================================

#ifndef FR_FREE_SURFACE_H
#define FR_FREE_SURFACE_H

#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/motion_functions/ChFunction.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"


namespace frydom{

    /// Pure Virtual Base class for a free surface system.
    class FrFreeSurface {

      public:

        /// Class constructor
        FrFreeSurface(double p_mean_height);

        virtual ~FrFreeSurface() {};

        /// Update the state of the free surface at the specified time.
        virtual void Synchronize(double time) {}

        /// Advance the state of the free surface by the specified duration.
        virtual void Advance(double step) {}

        /// Get the mean height of the free surface's plane.
        virtual double getMeanHeight() const;

        /// Get the free surface elevation at specified.
        virtual double GetHeight(double x, double y, double t) const = 0;

        /// Initializes the free surface system
        /// In any case, a mesh grid is used.
        /// this version concerns a rectangular grid
        void Initialize(double xmin,
                        double xmax,
                        double dx,
                        double ymin,
                        double ymax,
                        double dy
                        );

        /// Initializes the free surface system
        /// In any case, a mesh grid is used.
        /// this version concerns a square grid
        void Initialize(double lmin,
                        double lmax,
                        double dl
                        );

      protected:
        FrFreeSurface() {};

        double m_mean_height;
        chrono::ChCoordsys<> plane;  /// The reference plane of the free surface
        chrono::geometry::ChTriangleMeshConnected mesh;
    };

}  // end namespace frydom

#endif
