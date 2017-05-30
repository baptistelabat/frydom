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


#define FR_FLAT_FREE_SURFACE_H

#include "FrFreeSurface.h"

namespace frydom {
namespace environment{

    class FrOffshoreSystem;

    class FrFlatFreeSurface : public FrFreeSurface {  // le public permet de rendre les methodes de la classe de base publiques

      public:
        FrFlatFreeSurface(double p_mean_height);

        FrFlatFreeSurface(std::shared_ptr<FrOffshoreSystem> system,
                          double p_mean_height);

        /// Get the free surface elevation at specified (x,y) location.
        /// Currently returns the mean height passed at construction
        virtual double GetHeight(double x, double y, double t) const;


      protected:
        FrFlatFreeSurface() {};
    };

} // end namespace environment
} // end namespace frydom
