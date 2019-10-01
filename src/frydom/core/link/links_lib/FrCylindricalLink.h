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


#ifndef FRYDOM_FRCYLINDRICALLINK_H
#define FRYDOM_FRCYLINDRICALLINK_H

#include "FrLink.h"

namespace frydom {


    /**
     * \class FrCylindricalLink
     * \brief Class implementing a cylindrical link, derived from FrLink: allows rotation and translation around the Z axis.
     */
    class FrCylindricalLink : public FrLink {

     public:

      /// Constructor from two nodes and a pointer to the system.
      /// It automatically adds the link to the system
      FrCylindricalLink(const std::string &name,
                        FrOffshoreSystem *system,
                        const std::shared_ptr<FrNode> &node1,
                        const std::shared_ptr<FrNode> &node2);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "CylindricalLink"; }

    };

    /// Helper function to make it easy to link two nodes by a cylindrical link
    std::shared_ptr<FrCylindricalLink>
    make_cylindrical_link(const std::string &name,
                          FrOffshoreSystem *system,
                          const std::shared_ptr<FrNode> &node1,
                          const std::shared_ptr<FrNode> &node2);

}  // end namespace frydom

#endif //FRYDOM_FRCYLINDRICALLINK_H
