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


#include "FrAsset.h"

namespace frydom {

    namespace internal {

        FrAssetBase_::FrAssetBase_(FrAsset * asset) : m_frydomAsset(asset) {

        }

        void FrAssetBase_::Update(chrono::ChPhysicsItem *updater, const chrono::ChCoordsys<> &coords) {
            m_frydomAsset->Update();
        }

    } // end namespace frydom::internal


    FrAsset::FrAsset() {
        m_chronoAsset = std::make_shared<internal::FrAssetBase_>(this);
    }

    std::shared_ptr<chrono::ChAsset> FrAsset::GetChronoAsset() {
        return m_chronoAsset;
    }

} // end namespace frydom
