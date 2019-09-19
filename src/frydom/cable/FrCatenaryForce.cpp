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

#include "FrCatenaryForce.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"


namespace frydom {

    template <typename OffshoreSystemType>
    void FrCatenaryForce<OffshoreSystemType>::Compute(double time) {

        Position relpos;
        Force ForceInWorld;

        // Get the line tension from the corresponding node
        switch (m_line_side) {
            case FrCatenaryLine<OffshoreSystemType>::LINE_START:
                ForceInWorld = m_line->GetStartingNodeTension(NWU);
                relpos = m_line->GetStartingNode()->GetNodePositionInBody(NWU);
                break;

            case FrCatenaryLine<OffshoreSystemType>::LINE_END:
                ForceInWorld = m_line->GetEndingNodeTension(NWU);
                relpos = m_line->GetEndingNode()->GetNodePositionInBody(NWU);
                break;
        }

        // Set the tension in the world reference frame and NWU frame convention
        this->SetForceTorqueInWorldAtPointInBody(ForceInWorld, Torque(), relpos, NWU);

    }


}  // end namespace frydom
