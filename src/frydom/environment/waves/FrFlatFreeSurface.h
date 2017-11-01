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
// Simple flat free surface (constant height)
//
// =============================================================================

#ifndef FRYDOM_FR_FLAT_FREE_SURFACE_H
#define FRYDOM_FR_FLAT_FREE_SURFACE_H

#include "FrFreeSurface.h"

namespace frydom {

    // Forward declaration
    class FrOffshoreSystem;

        class FrFlatFreeSurface : public FrFreeSurface {

          public:
            /// Default constructor
            FrFlatFreeSurface(double mean_height);

            /// Get the free surface elevation at specified (x,y) location.
            /// Currently returns the mean height passed at construction
            virtual double GetHeight(double x, double y, double t) const;

            virtual void Update(double mytime);

          protected:
            FrFlatFreeSurface() {};
        };

} // end namespace frydom

#endif //FRYDOM_FR_FLAT_FREE_SURFACE_H