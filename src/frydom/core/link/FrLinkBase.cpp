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

#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  FrLinkBase::FrLinkBase(const std::string &name,
                         const std::string &type_name,
                         FrOffshoreSystem *system,
                         const std::shared_ptr<FrNode> &node1,
                         const std::shared_ptr<FrNode> &node2) :
      FrLoggable(name, type_name, system),
      m_node1(node1),
      m_node2(node2),
      c_chrono_body_1(node1->GetBody()->GetChronoBody()),
      c_chrono_body_2(node2->GetBody()->GetChronoBody()) {}

  std::shared_ptr<FrNode> FrLinkBase::GetNode1() {
    return m_node1;
  }

  const std::shared_ptr<FrNode> FrLinkBase::GetNode1() const {
    return m_node1;
  }

  std::shared_ptr<FrNode> FrLinkBase::GetNode2() {
    return m_node2;
  }

  const std::shared_ptr<FrNode> FrLinkBase::GetNode2() const {
    return m_node2;
  }

  FrBody *FrLinkBase::GetBody1() {
    return m_node1->GetBody();
  }

  FrBody *FrLinkBase::GetBody2() {
    return m_node2->GetBody();
  }

  std::shared_ptr<chrono::ChBody> FrLinkBase::GetChronoBody1() {
    return c_chrono_body_1;
  }

  std::shared_ptr<chrono::ChBody> FrLinkBase::GetChronoBody2() {
    return c_chrono_body_2;
  }

//    FrOffshoreSystem *FrLinkBase::GetSystem() {
//      return m_system;
//    }
//
//    FrFrame FrLinkBase::GetTransformFromFrame2ToFrame1() const {
//        return m_node2->GetFrameInWorld().GetOtherFrameRelativeTransform_WRT_ThisFrame(m_node1->GetFrameInWorld());
//    }


}  // end namespace frydom
