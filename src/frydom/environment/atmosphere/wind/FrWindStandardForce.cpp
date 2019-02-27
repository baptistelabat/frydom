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


#include "FrWindStandardForce.h"

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/atmosphere/FrAtmosphereInc.h"


namespace frydom {

    FrWindStandardForce::FrWindStandardForce() : FrForce() { }

    void FrWindStandardForce::SetLateralArea(double lateralArea) {
        assert(lateralArea > FLT_EPSILON);
        m_lateralArea = lateralArea;
    }

    void FrWindStandardForce::SetTransverseArea(double transverseArea) {
        assert(transverseArea > FLT_EPSILON);
        m_transverseArea = transverseArea;
    }

    void FrWindStandardForce::SetXCenter(double xCenter) {
        m_xCenter = xCenter;
    }

    void FrWindStandardForce::SetLenghtBetweenPerpendicular(double lpp) {
        assert(lpp > FLT_EPSILON);
        m_lpp = lpp;
    }

    void FrWindStandardForce::Initialize() {
        if (m_transverseArea < FLT_EPSILON) throw FrException(" error value transverse area");
        if (m_lateralArea < FLT_EPSILON) throw FrException("error value lateral area");
        if (m_lpp < FLT_EPSILON) throw FrException("error value length between perpendicular");
    }

    void FrWindStandardForce::Update(double time) {

        Force force;
        Torque torque;

        auto rho = GetSystem()->GetEnvironment()->GetAtmosphere()->GetDensity();

        FrFrame FrameAtCOG = m_body->GetFrameAtCOG(NWU);

        auto bodyVelocity = m_body->GetVelocityInWorld(NWU);
        bodyVelocity.z()= 0.;

        Velocity fluxVelocityInBody = m_body->GetSystem()->GetEnvironment()->GetAtmosphere()->GetWind()
                ->GetRelativeVelocityInFrame(FrameAtCOG, bodyVelocity, NWU);

        fluxVelocityInBody = internal::SwapFrameConvention(fluxVelocityInBody);
        fluxVelocityInBody = -fluxVelocityInBody;   // Swap convention GOTO/COMEFROM;

        double alpha = fluxVelocityInBody.GetProjectedAngleAroundZ(RAD);
        alpha = Normalize_0_2PI(alpha);

        auto ak = 0.5 * rho * fluxVelocityInBody.squaredNorm();

        force.x() = -0.7 * ak * m_transverseArea * cos(alpha);
        force.y() = 0.9 * ak * m_lateralArea * sin(alpha);
        force.z() = 0.;

        if (alpha > M_PI) alpha = 2.*M_PI - alpha;
        auto m1 = 0.3 * (1. - 2. * alpha / M_PI);
        torque.x() = 0.;
        torque.y() = 0.;
        torque.z() = force.y() * m1 * m_lpp;

        // Build the projected rotation in the XoY plane.
        double phi, theta, psi;
        m_body->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
        auto bodyRotation = FrRotation(Direction(0.,0.,1.), psi, NWU);
        auto frame = FrFrame(m_body->GetCOGPositionInWorld(NWU), bodyRotation, NWU);

        auto worldForce = frame.ProjectVectorFrameInParent(force, NWU);
        auto worldTorque = frame.ProjectVectorFrameInParent(torque, NWU);

        SetForceTorqueInWorldAtPointInBody(worldForce, worldTorque, Position(m_xCenter, 0., 0.), NWU);
    }

    void FrWindStandardForce::StepFinalize() {
        FrForce::StepFinalize();
    }

}  // end namespace frydom
