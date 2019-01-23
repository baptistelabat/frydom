//
// Created by frongere on 23/01/19.
//

#include <chrono/physics/ChLinkLock.h>
#include "FrPrismaticLink.h"

//#include "frydom/core/common/FrNode.h"

//#include "chrono/physics/ChLinkLock.h"


namespace frydom {


    std::shared_ptr<FrPrismaticLink>
    make_prismatic_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) {
        return std::make_shared<FrPrismaticLink>(node1, node2, system);
    }


    FrPrismaticLink::FrPrismaticLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2,
                                     FrOffshoreSystem_ *system) : _FrLinkLockBase(node1, node2, system) {

        m_chronoLink = std::make_shared<chrono::ChLinkLockPrismatic>();





    }

}  // end namespace frydom
