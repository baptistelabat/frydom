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


namespace frydom{

    /// Base class for a free surface system.
    class FrFreeSurface {

      public:

        /// Class constructor
        FrFreeSurface(double p_mean_height);

        virtual ~FrFreeSurface() {};

        /// Get the mean height of the free surface's plane
        virtual double GetMeanHeight();

        /// Set the mean height of the free surface's plane
        virtual void SetMeanHeight(double p_mean_height);

        /// Get the free surface elevation at specified
        virtual double GetHeight(double x, double y, double t) = 0;


      protected:
        FrFreeSurface() {};

        chrono::ChCoordsys<> plane;  /// The reference plane of the free surface
        std::shared_ptr<chrono::ChFunction> height_function;

        double m_mean_height;




    };

}  // end namespace frydom

#endif
