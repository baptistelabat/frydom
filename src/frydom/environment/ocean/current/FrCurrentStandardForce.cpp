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


#include "FrCurrentStandardForce.h"

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/core/common/FrFrame.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "FrCurrent.h"

namespace frydom {

    void FrCurrentStandardForce::SetMaximumBreadth(double breadth) {
        assert(breadth > FLT_EPSILON);
        m_breadth = breadth;
    }

    void FrCurrentStandardForce::SetDraft(double draft) {
        assert(draft > FLT_EPSILON);
        m_draft = draft;
    }

    void FrCurrentStandardForce::SetLateralArea(double lateralArea) {
        assert(lateralArea > FLT_EPSILON);
        m_lateralArea = lateralArea;
    }

    void FrCurrentStandardForce::SetTransverseArea(double transverseArea) {
        assert(transverseArea > FLT_EPSILON);
        m_transverseArea = transverseArea;
    }

    void FrCurrentStandardForce::SetXCenter(double xCenter) {
        m_xCenter = xCenter;
    }

    void FrCurrentStandardForce::SetLengthBetweenPerpendicular(double lpp) {
        assert(lpp > FLT_EPSILON);
        m_lpp = lpp;
    }

    void FrCurrentStandardForce::Initialize() {
        if (m_transverseArea < FLT_EPSILON and m_draft > FLT_EPSILON and m_breadth > FLT_EPSILON) {
            m_transverseArea = m_draft * m_breadth;
        }
        if (m_lateralArea < FLT_EPSILON) throw FrException("error value lateral area");
        if (m_transverseArea < FLT_EPSILON) throw FrException("error value transverse area");
        if (m_lpp < FLT_EPSILON) throw  FrException("error value length between perpendicular");
    }

    void FrCurrentStandardForce::Compute(double time) {

        Force force;
        Torque torque;

        auto rho = GetSystem()->GetEnvironment()->GetOcean()->GetDensity();

        FrFrame FrameAtCOG = m_body->GetFrameAtCOG(NWU);

        auto bodyVelocity = m_body->GetVelocityInWorld(NWU);
        bodyVelocity.z() = 0.;

        Velocity fluxVelocityInBody =
                m_body->GetSystem()->GetEnvironment()->GetOcean()->GetCurrent()->GetRelativeVelocityInFrame(FrameAtCOG, bodyVelocity, NWU);

        fluxVelocityInBody = internal::SwapFrameConvention(fluxVelocityInBody);
        fluxVelocityInBody = -fluxVelocityInBody;       // Swap convention GOTO/COMEFROM

        double alpha = fluxVelocityInBody.GetProjectedAngleAroundZ(RAD);
        alpha = Normalize_0_2PI(alpha);

        auto ak = 0.5 * rho * fluxVelocityInBody.squaredNorm();

        force.x() = -0.07 * ak * m_transverseArea * cos(alpha);
        force.y() = 0.6 * ak * m_lateralArea * sin(alpha);
        force.z() = 0.;

        if (alpha > M_PI) alpha = 2.*M_PI - alpha;
        auto m1 = std::min(0.4 * (1. - 2.* alpha / M_PI), 0.25);
        auto m2 = std::max(m1, -0.2);
        torque.x() = 0.;
        torque.y() = 0.;
        torque.z() = force.y() * m2 * m_lpp;

        // Build the projected rotation in the XoY plane.
        double phi, theta, psi;
        m_body->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
        auto bodyRotation = FrRotation(Direction(0.,0.,1.), psi, NWU);
        auto frame = FrFrame(m_body->GetCOGPositionInWorld(NWU), bodyRotation, NWU);

        auto worldForce = frame.ProjectVectorFrameInParent(force, NWU);
        auto worldTorque = frame.ProjectVectorFrameInParent(torque, NWU);

        SetForceTorqueInWorldAtPointInBody(worldForce, worldTorque, Position(m_xCenter, 0., 0.), NWU);
    }


}  // end namespace frydom
