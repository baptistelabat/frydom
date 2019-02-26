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


#include "FrWindForce.h"

#include "MathUtils/Angles.h"

#include "frydom/core/common/FrUnits.h"
#include "frydom/IO/FrLoader.h"
#include "frydom/core/common/FrFrame.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/atmosphere/FrAtmosphere_.h"
#include "FrWind.h"
#include "frydom/core/body/FrBody.h"


namespace frydom {

    FrWindForce_::FrWindForce_(std::string yamlFile) {
        this->ReadTable(yamlFile);
    }

    void FrWindForce_::ReadTable(std::string yamlFile) {

        std::vector<double> angle, Cx, Cy, Cm;
        ANGLE_UNIT unit;

        LoadWindTableFromYaml(yamlFile, angle, Cx, Cy, Cm, unit);

        auto n = angle.size();

        assert(Cx.size() == n);
        assert(Cy.size() == n);
        assert(Cm.size() == n);

        if (unit == mathutils::DEG) {
            mathutils::deg2rad(angle);
        }

        m_table.SetX(angle);
        m_table.AddY("Cx", Cx);
        m_table.AddY("Cy", Cy);
        m_table.AddY("Cm", Cm);
    }

    void FrWindForce_::Update(double time) {

        FrFrame_ cogFrame = m_body->GetFrameAtCOG(NWU);
        Velocity cogWorldVel = m_body->GetCOGVelocityInWorld(NWU);

        Velocity cogRelVel = m_body->GetSystem()->GetEnvironment()->GetAtmosphere()->GetWind()->GetRelativeVelocityInFrame(cogFrame, cogWorldVel, NWU);

        double alpha = cogRelVel.GetProjectedAngleAroundZ(mathutils::RAD) + MU_PI;
        alpha = mathutils::Normalize_0_2PI(alpha);

        auto cx = m_table.Eval("Cx", alpha);
        auto cy = m_table.Eval("Cy", alpha);
        auto cz = m_table.Eval("Cm", alpha);

        double velSquare = cogRelVel.squaredNorm();

        auto fx = cx * velSquare;
        auto fy = cy * velSquare;
        auto mz = cz * velSquare;

        SetForceTorqueInBodyAtCOG(Force(fx, fy, 0.), Torque(0., 0., mz), NWU);

    }

    void FrWindForce_::StepFinalize() {
        FrForce_::StepFinalize();
    }


}  // end namespace frydom
