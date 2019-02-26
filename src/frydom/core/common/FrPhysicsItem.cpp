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
//
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChColorAsset.h"

#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {

    namespace internal {


        _FrPhysicsItemBase::_FrPhysicsItemBase(FrPhysicsItem_ *item) : m_frydomPhysicsItem(item) {}

        void _FrPhysicsItemBase::SetupInitial() {
        }

        void _FrPhysicsItemBase::Update(bool update_assets) {
            this->Update(ChTime, update_assets);
        }

        void _FrPhysicsItemBase::Update(double time, bool update_assets) {
            m_frydomPhysicsItem->Update(time);
            ChPhysicsItem::Update(time, update_assets);
        }

    }  // end namespace frydom::internal



    FrPhysicsItem_::FrPhysicsItem_() {
        m_chronoPhysicsItem = std::make_shared<internal::_FrPhysicsItemBase>(this);
    };

    FrOffshoreSystem_* FrPhysicsItem_::GetSystem() {
        return m_system;
    }

    void FrPhysicsItem_::SetName(const char *name) {
        m_chronoPhysicsItem->SetName(name);
    }

    std::string FrPhysicsItem_::GetName() const {
        return m_chronoPhysicsItem->GetNameString();
    }

    std::shared_ptr<chrono::ChPhysicsItem> FrPhysicsItem_::GetChronoPhysicsItem() const {
        return m_chronoPhysicsItem;
    }

    void FrPhysicsItem_::AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh) {
        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
        shape->SetMesh(*mesh);
        m_chronoPhysicsItem->AddAsset(shape);
    }

    void FrPhysicsItem_::SetColor(NAMED_COLOR colorName) {
        SetColor(FrColor(colorName));
    }

    void FrPhysicsItem_::SetColor(const FrColor& color) {
        auto colorAsset = std::make_shared<chrono::ChColorAsset>(
                chrono::ChColor(color.R, color.G, color.B));
        m_chronoPhysicsItem->AddAsset(colorAsset);
    }

    void FrPhysicsItem_::SetupInitial() {
        m_chronoPhysicsItem->SetupInitial();
        Initialize();
    }

}  // end namespace frydom
