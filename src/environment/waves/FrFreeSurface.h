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
#include "chrono/core/ChFunction.h"

namespace frydom{
namespace chrono{

/// Base class for a free surface system.
class FrFreeSurface {
  public:
    FrFreeSurface() {}
    FrFreeSurface(double height);
    
    virtual ~FrFreeSurface() {}

    void SetHeightFunction(ChFunction height_fcn);

    void SetHeight(double height);

    double GetHeight();



    /// Update the state of the free surface at the specified time.
    virtual void Synchronize(double time) {}

    /// Advance the state of the free surface system by the specified duration.
    virtual void Advance(double step) {}

    /// Get the free surface height at the specified (x, y) location.
    virtual double GetHeight(double x, double y) const = 0;

    /// Get the water pressure at the specified (x,y,z) location
    virtual double GetPressure(double x, double y, double z)

    /// get the water velocity at the specified (x,y,z) location
    virtual double GetVelocity(double x, double y, double z)
  
  private:
    ChCoordsys<> plane;  /// The reference plane of the free surface
    ChFunction height_function;
};

}  // end namespace chrono
}  // end namespace frydom


#endif