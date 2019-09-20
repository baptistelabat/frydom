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

    template<typename OffshoreSystemType>
    FrCylindricalLink<OffshoreSystemType>::FrCylindricalLink(const std::shared_ptr<FrNode<OffshoreSystemType>> &node1,
                                                             const std::shared_ptr<FrNode<OffshoreSystemType>> &node2,
                                                             FrOffshoreSystem<OffshoreSystemType> *system)
        : FrLink<OffshoreSystemType>(node1, node2, system) {
      this->m_chronoLink->SetLinkType(CYLINDRICAL);
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrCylindricalLink<OffshoreSystemType>>
    make_cylindrical_link(const std::shared_ptr<FrNode<OffshoreSystemType>> &node1,
                          const std::shared_ptr<FrNode<OffshoreSystemType>> &node2,
                          FrOffshoreSystem<OffshoreSystemType> *system) {
      auto link = std::make_shared<FrCylindricalLink>(node1, node2, system);
      system->AddLink(link);
      return link;
    }

}  // end namespace frydom
