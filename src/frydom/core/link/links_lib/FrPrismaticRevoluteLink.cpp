//
// Created by lletourn on 04/06/19.
//

#include "FrPrismaticRevoluteLink.h"

namespace frydom {

    FrPrismaticRevoluteLink::FrPrismaticRevoluteLink(const std::shared_ptr<frydom::FrNode> &node1,
                                                             const std::shared_ptr<frydom::FrNode> &node2,
                                                             frydom::FrOffshoreSystem *system) : FrLink(node1, node2,
                                                                                                        system) {
        m_chronoLink->SetLinkType(PRISMATICREVOLUTE);

    }

    std::shared_ptr<FrPrismaticRevoluteLink> make_prismatic_revolute_link(
            const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem* system) {
        auto link = std::make_shared<FrPrismaticRevoluteLink>(node1, node2, system);
        system->AddLink(link);
        return link;
    }

} // end namespace frydom