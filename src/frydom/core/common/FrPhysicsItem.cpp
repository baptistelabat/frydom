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

        template<typename OffshoreSystemType>
        FrPhysicsItemBase<OffshoreSystemType>::FrPhysicsItemBase(FrPhysicsItem<OffshoreSystemType> *item)
            : m_frydomPhysicsItem(item) {}

        template<typename OffshoreSystemType>
        void FrPhysicsItemBase<OffshoreSystemType>::SetupInitial() {
        }

        template<typename OffshoreSystemType>
        void FrPhysicsItemBase<OffshoreSystemType>::Update(double time, bool update_assets) {
          m_frydomPhysicsItem->Update(time);
          ChPhysicsItem::Update(time, update_assets);
        }

    }  // end namespace frydom::internal


    template<typename OffshoreSystemType>
    FrPhysicsItem<OffshoreSystemType>::FrPhysicsItem() {
      m_chronoPhysicsItem = std::make_shared<internal::FrPhysicsItemBase>(this);
    };

    template<typename OffshoreSystemType>
    FrOffshoreSystem<OffshoreSystemType> *FrPhysicsItem<OffshoreSystemType>::GetSystem() {
      return m_system;
    }

    template<typename OffshoreSystemType>
    bool FrPhysicsItem<OffshoreSystemType>::IsActive() const {
      return m_isActive;
    }

    template<typename OffshoreSystemType>
    void FrPhysicsItem<OffshoreSystemType>::SetActive(bool active) {
      m_isActive = active;
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<internal::FrPhysicsItemBase<OffshoreSystemType>>
    FrPhysicsItem<OffshoreSystemType>::GetChronoPhysicsItem() const {
      return m_chronoPhysicsItem;
    }

    template<typename OffshoreSystemType>
    void FrPhysicsItem<OffshoreSystemType>::SetupInitial() {
      m_chronoPhysicsItem->SetupInitial();
      Initialize();
    }

    template<typename OffshoreSystemType>
    void FrPhysicsItem<OffshoreSystemType>::Update(double time) {
      if (IsActive())
        Compute(time);
    }

}  // end namespace frydom
