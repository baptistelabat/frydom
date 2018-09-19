//
// Created by camille on 17/07/18.
//

#include "FrWindStandardForce.h"
#include "frydom/core/FrHydroBody.h"
#include "frydom/environment/FrEnvironment.h"

#include "frydom/environment/wind/FrWind.h"

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

}