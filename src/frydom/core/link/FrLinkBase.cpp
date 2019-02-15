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

#include "frydom/core/common/FrNode.h"


namespace frydom {

    FrLinkBase_::FrLinkBase_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) :
            m_node1(node1), m_node2(node2) {
        m_system = system;
    }

    FrNode_* FrLinkBase_::GetNode1() {
        return m_node1.get();
    }

    const FrNode_* FrLinkBase_::GetNode1() const {
        return m_node1.get();
    }

    FrNode_* FrLinkBase_::GetNode2() {
        return m_node2.get();
    }

    const FrNode_* FrLinkBase_::GetNode2() const {
        return m_node2.get();
    }

    FrBody_* FrLinkBase_::GetBody1() {
        return m_node1->GetBody();
    }

    FrBody_* FrLinkBase_::GetBody2() {
        return m_node2->GetBody();
    }
//
//    FrFrame_ FrLinkBase_::GetTransformFromFrame2ToFrame1() const {
//        return m_node2->GetFrameInWorld().GetOtherFrameRelativeTransform_WRT_ThisFrame(m_node1->GetFrameInWorld());
//    }


}  // end namespace frydom
