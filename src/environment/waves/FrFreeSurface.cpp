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

#include "FrFreeSurface.h"

namespace frydom {

    FrFreeSurface::FrFreeSurface(double p_mean_height) : m_mean_height(p_mean_height){
        plane.pos[1] = p_mean_height;  // The free surface plane reference has the altitude the mean FS height
    }

    double FrFreeSurface::getMeanHeight() const {
       return m_mean_height;
    }

    void FrFreeSurface::Initialize(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

    }

    void FrFreeSurface::Initialize(double center_x, double center_y, double radius, double dtheta, double dr) {

    }


}  // end namespace frydom