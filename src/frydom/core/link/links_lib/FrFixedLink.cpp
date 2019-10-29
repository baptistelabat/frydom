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


#include "FrFixedLink.h"
#include "frydom/logging/FrTypeNames.h"

namespace frydom {


  FrFixedLink::FrFixedLink(const std::string &name,
                           FrOffshoreSystem *system,
                           const std::shared_ptr<FrNode> &node1,
                           const std::shared_ptr<FrNode> &node2)
      : FrLink(name, TypeToString(this), system, node1, node2) {

    m_chronoLink->SetLinkType(FIXED_LINK);
  }

  std::shared_ptr<FrFixedLink>
  make_fixed_link(const std::string &name,
                  FrOffshoreSystem *system,
                  const std::shared_ptr<FrNode> &node1,
                  const std::shared_ptr<FrNode> &node2) {

    auto link = std::make_shared<FrFixedLink>(name, system, node1, node2);
    system->Add(link);
    return link;
  }


}  // end namespace frydom
