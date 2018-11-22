//
// Created by camille on 20/11/18.
//

#include "FrPhysicsItem.h"

namespace frydom {

    namespace internal {

        _FrPhysicsItemBase::_FrPhysicsItemBase(FrPhysicsItem_ *item) : m_frydomPhysicsItem(item) {}

        void _FrPhysicsItemBase::SetupInitial() {
            m_frydomPhysicsItem->Initialize();
        }

        void _FrPhysicsItemBase::Update(bool update_assets) {
            this->Update(ChTime, update_assets);
        }

        void _FrPhysicsItemBase::Update(double time, bool update_assets) {
            ChPhysicsItem::Update(time, update_assets);
            m_frydomPhysicsItem->Update(time);
        }

    }

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

}