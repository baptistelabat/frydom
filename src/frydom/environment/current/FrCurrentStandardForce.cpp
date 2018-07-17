//
// Created by camille on 17/07/18.
//

#include "FrCurrentStandardForce.h"

namespace frydom {


    FrCurrentStandardForce::FrCurrentStandardForce(const std::shared_ptr<FrHydroBody> mybody) {

        this->SetLpp(mybody->GetLpp());
        this->SetLateralArea(mybody->GetLateralUnderWaterArea());
        this->SetTransverseArea(mybody->GetTransverseUnderWaterArea());
        this->SetWaterDensity(mybody->GetSystem()->GetEnvironment()->GetWaterDensity());
    }

    void FrCurrentStandardForce::UpdateState() {

        auto mybody = dynamic_cast<FrHydroBody*>(GetBody());

        // current speed coming from direction
        auto alpha = mybody->GetCurrentRelativeAngle(NWU, RAD);

        // Relative current speed
        auto relative_velocity = mybody->GetCurrentRelativeVelocity(NWU);
        relative_velocity.z() = 0.;         // keep only horizontal components
        auto vel2 = relative_velocity.Length2();

        // amplitude coefficient
        auto ak = 0.5*m_rho*vel2;

        force.x() = -0.07 * ak * m_breadth * m_draft * cos(alpha);
        force.y() = 0.6 * ak * m_area * sin(alpha);

        auto m1 = min(0.4 * (1 - 2*alpha / CH_C_PI), 0.25);
        auto m2 = max(m1, -0.2);
        moment.z() = force.y() * (m_xc + m2 * m_Lpp);

        // force in global reference frame
        force = mybody->TransformLocalToParent(force);

    }



}