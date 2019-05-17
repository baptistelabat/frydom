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


#ifndef FRYDOM_FRPOINTONPLANECONSTRAINT_H
#define FRYDOM_FRPOINTONPLANECONSTRAINT_H

#include "frydom/core/link/links_lib/FrLink.h"

namespace frydom {

    /**
    * \class FrPointOnPlaneConstraint
    * \brief Class for adding a constraint in which the Z translation of the point is locked, with respect to the Z plane
     * (translation of the point allowed in XY plane directions, free rotations)
    */
    class FrPointOnPlaneConstraint : public FrLink {

    public:

        /// Constructor from two nodes and a pointer to the system.
        /// It automatically adds the link to the system
        FrPointOnPlaneConstraint(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem* system);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "PointOnPlaneConstraint"; }

    };

    /// Helper function to make it easy to add a pointOnPlane constraint between two nodes
    std::shared_ptr<FrPointOnPlaneConstraint> make_pointOnPlane_constraint(
            const std::shared_ptr<FrNode>& pointNode, const std::shared_ptr<FrNode>& planeNode, FrOffshoreSystem* system);


}  // end namespace frydom

#endif //FRYDOM_FRPOINTONPLANECONSTRAINT_H
