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


#ifndef FRYDOM_FRSPHERICALLINK_H
#define FRYDOM_FRSPHERICALLINK_H

#include "FrLink.h"

namespace frydom {

    /**
     * \class FrSphericalLink
     * \brief Class for defining a spherical link, derived from FrLink : allows all rotations, no translations
     */
    class FrSphericalLink : public FrLink {

     public:

      FrSphericalLink(const std::string &name,
                      FrOffshoreSystem *system,
                      const std::shared_ptr<FrNode> &node1,
                      const std::shared_ptr<FrNode> &node2);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "SphericalLink"; }

    };

    /// Helper function to make it easy to link two nodes by a spherical link
    std::shared_ptr<FrSphericalLink> make_spherical_link(const std::string &name,
                                                         FrOffshoreSystem *system,
                                                         const std::shared_ptr<FrNode> &node1,
                                                         const std::shared_ptr<FrNode> &node2);

}  // end namespace frydom

#endif //FRYDOM_FRSPHERICALLINK_H
