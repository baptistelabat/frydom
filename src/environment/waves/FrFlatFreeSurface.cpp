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

#include "FrFlatFreeSurface.h"
#include "../../core/FrOffshoreSystem.h"

namespace frydom {
    namespace environment{


        FrFlatFreeSurface::FrFlatFreeSurface(double mean_height) : FrFreeSurface(mean_height){}


        double FrFlatFreeSurface::GetHeight(double x, double y, double t) const{
            return m_mean_height;
        }

        void FrFlatFreeSurface::Update(double mytime) {
            ChTime = mytime;
            std::cout << "Updating Free surface at time " << mytime << std::endl;
        }

    }  // end namespace environment
}  // end namespace frydom