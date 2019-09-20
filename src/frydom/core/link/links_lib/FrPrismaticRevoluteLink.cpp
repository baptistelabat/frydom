//
// Created by lletourn on 04/06/19.
//

#include "FrPrismaticRevoluteLink.h"

namespace frydom {

    template<typename OffshoreSystemType>
    FrPrismaticRevoluteLink<OffshoreSystemType>::FrPrismaticRevoluteLink(
        const std::shared_ptr<frydom::FrNode<OffshoreSystemType>> &node1,
        const std::shared_ptr<frydom::FrNode<OffshoreSystemType>> &node2,
        frydom::FrOffshoreSystem<OffshoreSystemType> *system) :
        FrLink<OffshoreSystemType>(node1, node2, system) {

      this->m_chronoLink->SetLinkType(PRISMATICREVOLUTE);
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrPrismaticRevoluteLink<OffshoreSystemType>> make_prismatic_revolute_link(
        const std::shared_ptr<FrNode<OffshoreSystemType>> &node1,
        const std::shared_ptr<FrNode<OffshoreSystemType>> &node2, FrOffshoreSystem<OffshoreSystemType> *system) {

      auto link = std::make_shared<FrPrismaticRevoluteLink>(node1, node2, system);
      system->AddLink(link);
      return link;
    }

} // end namespace frydom
