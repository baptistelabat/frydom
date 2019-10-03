//
// Created by lletourn on 04/06/19.
//

#include "FrPrismaticRevoluteLink.h"

namespace frydom {

  FrPrismaticRevoluteLink::FrPrismaticRevoluteLink(const std::string &name,
                                                   FrOffshoreSystem *system,
                                                   const std::shared_ptr<frydom::FrNode> &node1,
                                                   const std::shared_ptr<frydom::FrNode> &node2) :
      FrLink(name, system, node1, node2) {

    m_chronoLink->SetLinkType(PRISMATICREVOLUTE);

  }

  std::shared_ptr<FrPrismaticRevoluteLink>
  make_prismatic_revolute_link(const std::string &name,
                               FrOffshoreSystem *system,
                               const std::shared_ptr<FrNode> &node1,
                               const std::shared_ptr<FrNode> &node2) {

    auto link = std::make_shared<FrPrismaticRevoluteLink>(name, system, node1, node2);
    system->Add(link);
    return link;
  }

} // end namespace frydom
