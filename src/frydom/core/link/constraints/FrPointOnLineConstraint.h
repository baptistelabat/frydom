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


#ifndef FRYDOM_FRPOINTONLINECONSTRAINT_H
#define FRYDOM_FRPOINTONLINECONSTRAINT_H

#include "frydom/core/link/links_lib/FrLink.h"

namespace frydom {

    /**
    * \class FrPointOnLineConstraint
    * \brief Class for adding a constraint in which the point follow a line defined by the X axis of the line node
     * (translation of the point allowed along the X axis of the line node, free rotations)
    */
    class FrPointOnLineConstraint : public FrLink {

    public:

        /// Constructor from two nodes and a pointer to the system.
        /// It automatically adds the link to the system
        FrPointOnLineConstraint(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem* system);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "PointOnLineConstraint"; }

    };

    /// Helper function to make it easy to add a pointOnLine constraint between two nodes
    std::shared_ptr<FrPointOnLineConstraint> make_pointOnLine_constraint(
            const std::shared_ptr<FrNode>& pointNode, const std::shared_ptr<FrNode>& lineNode, FrOffshoreSystem* system);


}  // end namespace frydom

#endif //FRYDOM_FRPOINTONLINECONSTRAINT_H
