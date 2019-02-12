// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#include "FrLinkBase.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"


namespace frydom {

    FrLinkBase_::FrLinkBase_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) :
            m_node1(node1), m_node2(node2) {
        m_system = system;
    }

    std::shared_ptr<FrNode_> FrLinkBase_::GetNode1() {
        return m_node1;
    }

    const std::shared_ptr<FrNode_> FrLinkBase_::GetNode1() const {
        return m_node1;
    }

    std::shared_ptr<FrNode_> FrLinkBase_::GetNode2() {
        return m_node2;
    }

    const std::shared_ptr<FrNode_> FrLinkBase_::GetNode2() const {
        return m_node2;
    }

    FrBody_* FrLinkBase_::GetBody1() {
        return m_node1->GetBody();
    }

    FrBody_* FrLinkBase_::GetBody2() {
        return m_node2->GetBody();
    }

    std::shared_ptr<chrono::ChBody> FrLinkBase_::GetChronoBody1() {
        return GetBody1()->GetChronoBody();
    }

    std::shared_ptr<chrono::ChBody> FrLinkBase_::GetChronoBody2() {
        return GetBody1()->GetChronoBody();
    }
//
//    FrFrame_ FrLinkBase_::GetTransformFromFrame2ToFrame1() const {
//        return m_node2->GetFrameInWorld().GetOtherFrameRelativeTransform_WRT_ThisFrame(m_node1->GetFrameInWorld());
//    }


}  // end namespace frydom
