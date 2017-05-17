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

#ifndef FR_FLAT_FREE_SURFACE_H
#define FR_FLAT_FREE_SURFACE_H

#include "FrFreeSurface.h"

namespace frydom {
namespace chrono {

class FrFlatFreeSurface : public FrFreeSurface {
  public:
    /// Construct a default flat free surface
    FrFlatFreeSurface();

    FrFlatFreeSurface(double height  /// [meter] terrain height
        );
    ~FlatTerrain() {}

    /// Get the free surface height at the specified (x,y) location.
    /// Returns the constant value passed at construction.
    virtual double GetHeight(double x, double y) const { return fs_height; }

    /// Get the water pressure at the specified (x,y,z) location.
    /// TODO
    virtual double GetPressure(double x, double y, double z);

    /// Get the water velocity at the specified (x,y,z) location.
    /// TODO
    virtual double GetVelocity(double x, double y, double z) const { return 0.; }

  private:
    double fs_height;
    
};


} // end namesapace chrono
} // end namespace frydom