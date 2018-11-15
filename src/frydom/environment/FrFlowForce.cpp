//
// Created by camille on 15/11/18.
//

#include "FrFlowForce.h"

#include "frydom/core/FrBody.h"
#include "frydom/environment/FrFlowBase.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/IO/FrLoader.h"
#include "frydom/core/FrFrame.h"


namespace frydom {

    FrFlowForce::FrFlowForce(const std::string& yamlFile) {
        this->ReadTable(yamlFile);
    }

    void FrFlowForce::ReadTable(const std::string& yamlFile) {

        std::vector<double> angle, cx, cy, cn;
        ANGLE_UNIT angle_unit;

        LoadFlowPolarCoeffFromYaml(yamlFile, angle, cx, cy, cn, angle_unit);

        auto n = angle.size();

        assert(cx.size() == n);
        assert(cy.size() == n);
        assert(cn.size() == n);

        m_table.SetX(angle);
        m_table.AddY("cx", cx);
        m_table.AddY("cz", cy);
        m_table.AddY("cn", cn);
    }

    void FrFlowForce::Update(double time) {

        FrFrame_ FrameAtCOG = m_body->GetFrameAtCOG(NWU);
        Velocity VelocityInWorldAtCOG =  m_body->GetCOGVelocityInWorld(NWU);

        Velocity FluxVelocityInFrame = m_body->GetSystem()->GetEnvironment()->GetWind()
                ->GetRelativeVelocityInFrame(FrameAtCOG, VelocityInWorldAtCOG, NWU);

        double alpha = FluxVelocityInFrame.GetProjectedAngleAroundZ(RAD)+M_PI;
        alpha = Normalize_0_2PI(alpha);

        auto cx = m_table.Eval("cx", alpha);
        auto cy = m_table.Eval("cy", alpha);
        auto cn = m_table.Eval("cn", alpha);

        double SquaredVelocity = FluxVelocityInFrame.squaredNorm();

        auto fx = cx * SquaredVelocity;
        auto fy = cy * SquaredVelocity;
        auto mz = cn * SquaredVelocity;

        SetForceTorqueInBodyAtCOG(Force(fx, fy, 0.), Torque(0., 0., mz), NWU);

    }

} // end of namespace frydom