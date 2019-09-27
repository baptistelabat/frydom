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


#include "FrPhysicsItem.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrNodeAsset.h"

#include "frydom/asset/FrAsset.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {

    namespace internal {


        FrPhysicsItemBase::FrPhysicsItemBase(FrPhysicsItem *item) : m_frydomPhysicsItem(item) {}

        void FrPhysicsItemBase::SetupInitial() {
        }

        void FrPhysicsItemBase::Update(double time, bool update_assets) {
            m_frydomPhysicsItem->Update(time);
            ChPhysicsItem::Update(time, update_assets);
        }

        void FrPhysicsItemBase::RemoveAsset(std::shared_ptr<chrono::ChAsset> asset) {
            assert(std::find<std::vector<std::shared_ptr<chrono::ChAsset>>::iterator>(assets.begin(), assets.end(),
                    asset) != assets.end());

            auto it = std::find(assets.begin(), assets.end(), asset);
            if (it != assets.end())
                assets.erase(it);

            RemoveIrrNodeAsset();
        }

        void FrPhysicsItemBase::RemoveIrrNodeAsset() {

            std::shared_ptr<chrono::irrlicht::ChIrrNodeAsset> myirrasset;

            for (unsigned int k = 0; k < assets.size(); k++) {
                std::shared_ptr<chrono::ChAsset> k_asset = assets[k];
                myirrasset = std::dynamic_pointer_cast<chrono::irrlicht::ChIrrNodeAsset>(k_asset);
            }

            if (myirrasset) {
                auto it = std::find(assets.begin(), assets.end(), myirrasset);
                if (it != assets.end())
                    assets.erase(it);
            }

        }

    }  // end namespace frydom::internal



    FrPhysicsItem::FrPhysicsItem() {
        m_chronoPhysicsItem = std::make_shared<internal::FrPhysicsItemBase>(this);
    };

    FrOffshoreSystem* FrPhysicsItem::GetSystem() {
        return m_system;
    }

    bool FrPhysicsItem::IsActive() const {
        return m_isActive;
    }

    void FrPhysicsItem::SetActive(bool active) {
        m_isActive = active;
    }

    std::shared_ptr<internal::FrPhysicsItemBase> FrPhysicsItem::GetChronoPhysicsItem() const {
        return m_chronoPhysicsItem;
    }

    void FrPhysicsItem::SetupInitial() {
        m_chronoPhysicsItem->SetupInitial();
        Initialize();
    }

    void FrPhysicsItem::Update(double time) {
        if(IsActive())
            Compute(time);
    }

}  // end namespace frydom
