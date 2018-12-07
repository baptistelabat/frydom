//
// Created by camille on 17/07/18.
//

#include "FrWindStandardForce.h"
#include "frydom/core/FrHydroBody.h"
#include "frydom/environment/FrEnvironment.h"

#include "frydom/environment/atmosphere/FrAtmosphere_.h"
#include "frydom/environment/flow/FrFlowBase.h"

#include "FrWind.h"

namespace frydom {


    FrWindStandardForce::FrWindStandardForce(const std::shared_ptr<FrHydroBody> mybody) {
        this->SetLateralArea(mybody->GetLateralAboveWaterArea());
        this->SetTransverseArea(mybody->GetTransverseAboveWaterArea());
        this->SetLpp(mybody->GetLpp());
        this->SetAirDensity(mybody->GetSystem()->GetEnvironment()->GetAirDensity());
    }


    void FrWindStandardForce::UpdateState() {

        auto mybody = dynamic_cast<FrHydroBody*>(GetBody());

        auto body_velocity = mybody->GetVelocity();
        auto wind_velocity = mybody->GetSystem()->GetEnvironment()->GetWind()->GetFluxVector(NWU);

        // Relative wind speed
        auto relative_velocity = body_velocity - wind_velocity;
        relative_velocity = mybody->Dir_World2Body(relative_velocity);

        auto vx = relative_velocity.x();
        auto vy = relative_velocity.y();
        auto vel2 = vx*vx + vy*vy;

        // Relative wind angle
        auto alpha = atan2(vy, vx);
        alpha = Normalize__PI_PI(alpha);

        // amplitude coefficient
        auto ak = 0.5*m_rho_air*vel2;

        force.x() = -0.7 * ak * m_transverse_area * cos(alpha);
        force.y() = 0.9 * ak * m_lateral_area * sin(alpha);

        auto m1 = 0.3 * (1. - 2.*std::abs(alpha) / chrono::CH_C_PI);
        moment.z() = force.y() * (m_xc + m1 * m_lpp);

        // force in global reference frame
        force = mybody->Dir_Body2World(force);

    }























    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    FrWindStandardForce_::FrWindStandardForce_() : FrForce_() { }

    void FrWindStandardForce_::SetLateralArea(double lateralArea) {
        assert(lateralArea > FLT_EPSILON);
        m_lateralArea = lateralArea;
    }

    void FrWindStandardForce_::SetTransverseArea(double transverseArea) {
        assert(transverseArea > FLT_EPSILON);
        m_transverseArea = transverseArea;
    }

    void FrWindStandardForce_::SetXCenter(double xCenter) {
        m_xCenter = xCenter;
    }

    void FrWindStandardForce_::SetLenghtBetweenPerpendicular(double lpp) {
        assert(lpp > FLT_EPSILON);
        m_lpp = lpp;
    }

    void FrWindStandardForce_::Initialize() {
        if (m_transverseArea < FLT_EPSILON) throw FrException(" error value transverse area");
        if (m_lateralArea < FLT_EPSILON) throw FrException("error value lateral area");
        if (m_lpp < FLT_EPSILON) throw FrException("error value length between perpendicular");
    }

    void FrWindStandardForce_::Update(double time) {

        Force force;
        Torque torque;

        auto rho = GetSystem()->GetEnvironment()->GetAtmosphere()->GetDensity();

        FrFrame_ FrameAtCOG = m_body->GetFrameAtCOG(NWU);
        Velocity VelocityInWorldAtCOG = m_body->GetCOGVelocityInWorld(NWU);

        Velocity fluxVelocityInBody = m_body->GetSystem()->GetEnvironment()->GetAtmosphere()->GetWind()
                ->GetRelativeVelocityInFrame(FrameAtCOG, VelocityInWorldAtCOG, NED);

        double alpha = fluxVelocityInBody.GetProjectedAngleAroundZ(RAD);
        alpha = Normalize__PI_PI(alpha);

        auto ak = 0.5 * rho * fluxVelocityInBody.squaredNorm();

        force.x() = -0.7 * ak * m_transverseArea * cos(alpha);
        force.y() = 0.9 * ak * m_lateralArea * sin(alpha);
        force.z() = 0.;
        SetForceInBody(force, NWU);

        auto m1 = 0.3 * (1. - 2. * alpha / M_PI);
        torque.x() = 0.;
        torque.y() = 0.;
        torque.z() = force.y() * (m1 * m_lpp - m_xCenter);
        SetTorqueInBodyAtCOG(torque, NWU);
    }

    void FrWindStandardForce_::StepFinalize() {

    }







}