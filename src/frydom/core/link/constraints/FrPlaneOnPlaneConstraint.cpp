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


#include "FrPlaneOnPlaneConstraint.h"


namespace frydom {

    FrPlaneOnPlaneConstraint::FrPlaneOnPlaneConstraint(const std::shared_ptr<FrNode> &node1,
                                                         const std::shared_ptr<FrNode> &node2,
                                                         FrOffshoreSystem *system) : FrLink(node1, node2, system) {
        m_chronoLink->SetLinkType(PLANEONPLANE);
    }

    std::shared_ptr<FrPlaneOnPlaneConstraint> make_planeOnPlane_constraint(
            const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem* system) {
        auto constraint = std::make_shared<FrPlaneOnPlaneConstraint>(node1, node2, system);
        system->AddLink(constraint);
        return constraint;
    }


}  // end namespace frydom
