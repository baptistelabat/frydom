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


#ifndef FRYDOM_FRASSETBUOY_H
#define FRYDOM_FRASSETBUOY_H


#include "FrAssetComponent.h"


// Forward declaration
namespace chrono {

    template <typename Scalar>
    class ChVector;

    class ChColorAsset;
    class ChSphereShape;
}

namespace frydom {

    /**
     * \class FrAssetBuoy
     * \brief Class for the buoy assets.
     */
    class FrAssetBuoy : public FrAssetComponent {

        // FIXME : ne pas reposer sur les classes chrono !!!

    public:

        FrAssetBuoy(chrono::ChVector<double> mPos, double mRadius, chrono::ChColor mColor);

        FrAssetBuoy(double mRadius, chrono::ChColor mColor);

        explicit FrAssetBuoy(double mRadius);

        FrAssetBuoy();

        void SetColorAsset(std::shared_ptr<chrono::ChColorAsset> color); // FIXME : utiliser FrColor !!

        void SetShapeAsset(std::shared_ptr<chrono::ChSphereShape> shape); // FIXME : pourquoi public ?

    };

} // end namespace frydom

#endif //FRYDOM_FRASSETBUOY_H
