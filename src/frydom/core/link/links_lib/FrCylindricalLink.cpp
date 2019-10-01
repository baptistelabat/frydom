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


#include "FrCylindricalLink.h"


namespace frydom {

    FrCylindricalLink::FrCylindricalLink(const std::string &name,
                                         FrOffshoreSystem *system,
                                         const std::shared_ptr<FrNode> &node1,
                                         const std::shared_ptr<FrNode> &node2)
        : FrLink(name, system, node1, node2) {
      m_chronoLink->SetLinkType(CYLINDRICAL);
    }

    std::shared_ptr<FrCylindricalLink>
    make_cylindrical_link(const std::string &name,
                          FrOffshoreSystem *system,
                          const std::shared_ptr<FrNode> &node1,
                          const std::shared_ptr<FrNode> &node2) {
      auto link = std::make_shared<FrCylindricalLink>(name, system, node1, node2);
      system->AddLink(link);
      return link;
    }

}  // end namespace frydom
