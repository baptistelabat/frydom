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


#ifndef FRYDOM_FRFIXEDLINK_H
#define FRYDOM_FRFIXEDLINK_H

#include "FrLink.h"

namespace frydom {

    /**
     * \class FrFixedLink
     * \brief Class for implementing a fixed kinematic link, derived from FrLink: no translation or rotation allowed
     * between the bodies
     */
    template<typename OffshoreSystemType>
    class FrFixedLink : public FrLink<OffshoreSystemType> {


     public:

      /// Constructor from two nodes and a pointer to the system.
      /// It automatically adds the link to the system
      FrFixedLink(const std::shared_ptr<FrNode<OffshoreSystemType>> &node1,
                  const std::shared_ptr<FrNode<OffshoreSystemType>> &node2,
                  FrOffshoreSystem<OffshoreSystemType> *system);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "FixedLink"; }

    };


    /// Helper function to make it easy to link two nodes by a fixed link
    template<typename OffshoreSystemType>
    std::shared_ptr<FrFixedLink<OffshoreSystemType>>
    make_fixed_link(const std::shared_ptr<FrNode<OffshoreSystemType>> &node1,
                    const std::shared_ptr<FrNode<OffshoreSystemType>> &node2,
                    FrOffshoreSystem<OffshoreSystemType> *system);
}  // end namespace frydom

#endif //FRYDOM_FRFIXEDLINK_H
