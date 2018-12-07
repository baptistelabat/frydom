//
// Created by Lucas Letournel on 05/12/18.
//

#ifndef FRYDOM_FRFREESURFACEPHYSICITEM_H
#define FRYDOM_FRFREESURFACEPHYSICITEM_H

#include "frydom/asset/FrGridAsset.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"

namespace frydom {

    class FrFreeSurface_;
    class FrTriangleMeshConnected;

    class FrFreeSurfaceGridAsset : public FrGridAsset{

    private:
        /// Pointer to the free surface containing this asset
        FrFreeSurface_* m_freeSurface;

    public:
        /// Default constructor
        /// \param freeSurface free surface containing this asset
        explicit FrFreeSurfaceGridAsset(FrFreeSurface_* freeSurface);

        /// FrFreeSurfaceGridAsset update method
        /// \param time time of the simulation
        void Update(double time) override;

    };
}

#endif //FRYDOM_FRFREESURFACEPHYSICITEM_H
