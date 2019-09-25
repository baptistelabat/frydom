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


namespace frydom {


    FrFixedLink::FrFixedLink(const std::string &&name,
                             const std::shared_ptr<FrNode> &node1,
                             const std::shared_ptr<FrNode> &node2,
                             FrOffshoreSystem *system)
        : FrLink(std::move(name), node1, node2, system) {
      m_chronoLink->SetLinkType(FIXED_LINK);
    }

    std::shared_ptr<FrFixedLink>
    make_fixed_link(const std::string &&name,
                    const std::shared_ptr<FrNode> &node1,
                    const std::shared_ptr<FrNode> &node2,
                    FrOffshoreSystem *system) {
      auto link = std::make_shared<FrFixedLink>(std::move(name), node1, node2, system);
      system->AddLink(link);
      return link;
    }
}  // end namespace frydom
