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

#include "frydom/asset/FrAsset.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {

    namespace internal {


        FrPhysicsItemBase::FrPhysicsItemBase(FrPhysicsItem *item) : m_frydomPhysicsItem(item) {}

        void FrPhysicsItemBase::SetupInitial() {
        }

        void FrPhysicsItemBase::Update(bool update_assets) {
            this->Update(ChTime, update_assets);
        }

        void FrPhysicsItemBase::Update(double time, bool update_assets) {
            m_frydomPhysicsItem->Update(time);
            ChPhysicsItem::Update(time, update_assets);
        }

    }  // end namespace frydom::internal



    FrPhysicsItem::FrPhysicsItem() {
        m_chronoPhysicsItem = std::make_shared<internal::FrPhysicsItemBase>(this);
    };

    FrOffshoreSystem* FrPhysicsItem::GetSystem() {
        return m_system;
    }

    void FrPhysicsItem::SetName(const char *name) {
        m_chronoPhysicsItem->SetName(name);
    }

    std::string FrPhysicsItem::GetName() const {
        return m_chronoPhysicsItem->GetNameString();
    }

    std::shared_ptr<chrono::ChPhysicsItem> FrPhysicsItem::GetChronoPhysicsItem() const {
        return m_chronoPhysicsItem;
    }

    void FrPhysicsItem::SetupInitial() {
        m_chronoPhysicsItem->SetupInitial();
        Initialize();
    }

    void FrPhysicsItem::StepFinalize() {

        FrAssetOwner::UpdateAsset();

    }

}  // end namespace frydom
