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


#include "FrCurrentForce.h"
#include "frydom/environment/flow/FrFlowBase.h"

#include "chrono/physics/ChBody.h"

#include "frydom/core/common/FrConvention.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/IO/FrLoader.h"

#include "frydom/environment/FrEnvironmentInc.h"


namespace frydom {

    FrCurrentForce_::FrCurrentForce_(std::string yamlFile) {
        m_coeffsTable = std::move(MakeCurrentPolarCoeffTable(yamlFile));
    }

    void FrCurrentForce_::Update(double time) {

        FrFrame_ cogFrame = m_body->GetFrameAtCOG(NWU);
        Velocity cogWorldVel = m_body->GetCOGVelocityInWorld(NWU);

        Velocity cogRelVel = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetCurrent()->GetRelativeVelocityInFrame(cogFrame, cogWorldVel, NWU);

        double alpha = cogRelVel.GetProjectedAngleAroundZ(DEG)+180.;
        alpha = Normalize__180_180(alpha);

        auto cx = m_coeffsTable.CX(alpha, NWU);
        auto cy = m_coeffsTable.CY(alpha, NWU);
        auto cz = m_coeffsTable.CZ(alpha, NWU);

        double velSquare = cogRelVel.squaredNorm();

        auto fx = -cx * velSquare;
        auto fy = -cy * velSquare;
        auto mz = -cz * velSquare;

        SetForceTorqueInBodyAtCOG(Force(fx, fy, 0.), Torque(0., 0., mz), NWU);

    }

    void FrCurrentForce_::StepFinalize() {
        FrForce_::StepFinalize();
    }


}  // end namespace frydom
