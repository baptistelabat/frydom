//
// Created by lletourn on 04/06/19.
//

#ifndef FRYDOM_FRPRISMATICREVOLUTELINK_H
#define FRYDOM_FRPRISMATICREVOLUTELINK_H

#include "FrLink.h"


namespace frydom {

    /**
     * \class FrPrismaticRevoluteLink
     * \brief Class for defining a composition between a prismatic link along X axis, and a revolute link around Z axis.
     * Derived from FrLink : allows translation along X axis and rotation around Z axis.
     */
    class FrPrismaticRevoluteLink : public FrLink {

     public:

      FrPrismaticRevoluteLink(const std::string &name,
                              FrOffshoreSystem *system,
                              const std::shared_ptr<FrNode> &node1,
                              const std::shared_ptr<FrNode> &node2);

    };

    /// Helper function to make it easy to link two nodes by a revolute-prismatic link
    std::shared_ptr<FrPrismaticRevoluteLink> make_prismatic_revolute_link(const std::string &name,
                                                                          FrOffshoreSystem *system,
                                                                          const std::shared_ptr<FrNode> &node1,
                                                                          const std::shared_ptr<FrNode> &node2);

} // end namespace frydom

#endif //FRYDOM_FRPRISMATICREVOLUTELINK_H
