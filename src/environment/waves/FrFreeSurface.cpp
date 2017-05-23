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

    FrFreeSurface::FrFreeSurface(double p_mean_height) : m_mean_height(p_mean_height) {
    }

    double FrFreeSurface::GetMeanHeight() {
       return m_mean_height;
    }

    void FrFreeSurface::SetMeanHeight(double p_mean_height) {
        m_mean_height = p_mean_height;
    }


}  // end namespace frydom