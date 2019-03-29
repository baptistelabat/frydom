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

    bool FrPhysicsItem::IsActive() const {
        return m_isActive;
    }

    void FrPhysicsItem::SetActive(bool active) {
        m_isActive = active;
    }

    internal::FrPhysicsItemBase *FrPhysicsItem::GetChronoItem_ptr() const {
        return m_chronoPhysicsItem.get();
    }

    std::shared_ptr<internal::FrPhysicsItemBase> FrPhysicsItem::GetChronoPhysicsItem() const {
        return m_chronoPhysicsItem;
    }

    void FrPhysicsItem::SetupInitial() {
        m_chronoPhysicsItem->SetupInitial();
        Initialize();
    }

    void FrPhysicsItem::StepFinalize() {

        FrAssetOwner::UpdateAsset();

    }

    void FrPhysicsItem::Update(double time) {
        if(IsActive())
            Compute(time);
    }

}  // end namespace frydom
