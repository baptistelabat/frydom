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


#ifndef FRYDOM_FRPERPENDICULARCONSTRAINT_H
#define FRYDOM_FRPERPENDICULARCONSTRAINT_H

#include "frydom/core/link/links_lib/FrLink.h"

namespace frydom {

    /**
    * \class FrPerpendicularConstraint
    * \brief Class for adding a constraint in which the Y axes of the two nodes are orthogonal.
     * (translation allowed in all directions, rotation allowed only around Y axes)
    */
    class FrPerpendicularConstraint : public FrLink {

    public:

        /// Constructor from two nodes and a pointer to the system.
        /// It automatically adds the link to the system
        FrPerpendicularConstraint(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem* system);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "PerpendicularConstraint"; }

    };

    /// Helper function to make it easy to add a perpendicular constraint between two nodes
    std::shared_ptr<FrPerpendicularConstraint> make_perpendicular_constraint(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem* system);


}  // end namespace frydom

#endif //FRYDOM_FRPERPENDICULARCONSTRAINT_H
