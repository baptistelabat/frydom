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


#include "FrLinkBase.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrLinkBase<OffshoreSystemType>::FrLinkBase(const std::shared_ptr<FrNode<OffshoreSystemType>> &node1,
                                               const std::shared_ptr<FrNode<OffshoreSystemType>> &node2,
                                               FrOffshoreSystem<OffshoreSystemType> *system) :
        m_node1(node1), m_node2(node2), m_system(system) {
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrNode<OffshoreSystemType>> FrLinkBase<OffshoreSystemType>::GetNode1() {
      return m_node1;
    }

    template<typename OffshoreSystemType>
    const std::shared_ptr<FrNode<OffshoreSystemType>> FrLinkBase<OffshoreSystemType>::GetNode1() const {
      return m_node1;
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrNode<OffshoreSystemType>> FrLinkBase<OffshoreSystemType>::GetNode2() {
      return m_node2;
    }

    template<typename OffshoreSystemType>
    const std::shared_ptr<FrNode<OffshoreSystemType>> FrLinkBase<OffshoreSystemType>::GetNode2() const {
      return m_node2;
    }

    template<typename OffshoreSystemType>
    FrBody<OffshoreSystemType> *FrLinkBase<OffshoreSystemType>::GetBody1() {
      return m_node1->GetBody();
    }

    template<typename OffshoreSystemType>
    FrBody<OffshoreSystemType> *FrLinkBase<OffshoreSystemType>::GetBody2() {
      return m_node2->GetBody();
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<chrono::ChBody> FrLinkBase<OffshoreSystemType>::GetChronoBody1() {
      return GetBody1()->GetChronoBody();
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<chrono::ChBody> FrLinkBase<OffshoreSystemType>::GetChronoBody2() {
      return GetBody2()->GetChronoBody();
    }

    template<typename OffshoreSystemType>
    FrOffshoreSystem<OffshoreSystemType> *FrLinkBase<OffshoreSystemType>::GetSystem() {
      return m_system;
    }

}  // end namespace frydom
