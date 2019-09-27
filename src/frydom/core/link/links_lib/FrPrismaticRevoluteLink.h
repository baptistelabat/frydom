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

      FrPrismaticRevoluteLink(const std::string& name,
                              const std::shared_ptr<FrNode> &node1,
                              const std::shared_ptr<FrNode> &node2,
                              FrOffshoreSystem *system);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "PrismaticRevoluteLink"; }

    };

    /// Helper function to make it easy to link two nodes by a revolute-prismatic link
    std::shared_ptr<FrPrismaticRevoluteLink> make_prismatic_revolute_link(const std::string& name,
                                                                          const std::shared_ptr<FrNode> &node1,
                                                                          const std::shared_ptr<FrNode> &node2,
                                                                          FrOffshoreSystem *system);

} // end namespace frydom

#endif //FRYDOM_FRPRISMATICREVOLUTELINK_H
