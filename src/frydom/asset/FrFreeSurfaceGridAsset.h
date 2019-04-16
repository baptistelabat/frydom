// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef FRYDOM_FRFREESURFACEPHYSICITEM_H
#define FRYDOM_FRFREESURFACEPHYSICITEM_H

#include "frydom/asset/FrGridAsset.h"


namespace frydom {

    class FrFreeSurface;

    /**
     * \class FrFreeSurfaceGridAsset
     * \brief Class for displaying the free surface grid.
     */
    class FrFreeSurfaceGridAsset : public FrGridAsset {

    private:

        FrFreeSurface* m_freeSurface;    ///> Pointer to the free surface containing this asset

    public:
        /// Default constructor
        /// \param body body containing this asset (usually WorldBody)
        explicit FrFreeSurfaceGridAsset(FrFreeSurface* freeSurface);

        /// Update the state of the asset, at the end of a time step
        void StepFinalize() override;

    };

} // end namespace frydom

#endif //FRYDOM_FRFREESURFACEPHYSICITEM_H
