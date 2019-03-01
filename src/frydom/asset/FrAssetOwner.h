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

#ifndef FRYDOM_FRASSETOWNER_H
#define FRYDOM_FRASSETOWNER_H

#include "frydom/core/misc/FrColors.h"

namespace frydom {

    // Forward declarations:
    class FrAsset;
    class FrTriangleMeshConnected;

    class FrAssetOwner {

    protected:

        using AssetContainer = std::vector<std::shared_ptr<FrAsset>>;
        AssetContainer m_assets;                    ///< Container of the assets added to the body

        virtual chrono::ChPhysicsItem* GetChronoItem() const = 0;

    public:

        void UpdateAsset();

        /// Add an asset for visualization, based on FrAsset, to the asset owner.
        /// Check FrAsset for a list of all its subclasses.
        /// \param asset asset to be added
        void AddAsset(std::shared_ptr<FrAsset> asset);

        void SetColor(NAMED_COLOR colorName);

        void SetColor(const FrColor& color);

        // Linear iterators on assets
        using AssetIter = AssetContainer::iterator;
        using ConstAssetIter = AssetContainer::const_iterator;

        AssetIter       asset_begin();
        ConstAssetIter  asset_begin() const;

        AssetIter       asset_end();
        ConstAssetIter  asset_end() const;

    };

}// end namespace frydom
#endif //FRYDOM_FRASSETOWNER_H
