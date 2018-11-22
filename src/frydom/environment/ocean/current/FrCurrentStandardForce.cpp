//
// Created by camille on 17/07/18.
//

#include "FrCurrentStandardForce.h"
#include "frydom/core/FrHydroBody.h"

#include "frydom/environment/FrEnvironment.h"

namespace frydom {


    FrCurrentStandardForce::FrCurrentStandardForce(const std::shared_ptr<FrHydroBody> mybody) {

        this->SetLpp(mybody->GetLpp());
        this->SetLateralArea(mybody->GetLateralUnderWaterArea());
        this->SetTransverseArea(mybody->GetTransverseUnderWaterArea());
        this->SetWaterDensity(mybody->GetSystem()->GetEnvironment()->GetWaterDensity());
    }

    void FrCurrentStandardForce::SetDraft(const double draft) {
        m_draft = draft;
        m_transverse_area = m_draft * m_breadth;
    }

    void FrCurrentStandardForce::SetMaxBreadth(const double breadth) {
        m_breadth = breadth;
        m_transverse_area = m_draft * m_breadth;
    }

    void FrCurrentStandardForce::UpdateState() {

        auto mybody = dynamic_cast<FrHydroBody*>(GetBody());

        // current speed coming from direction
        auto alpha = mybody->GetCurrentRelativeAngle(NWU, RAD);
        alpha = Normalize__PI_PI(alpha);

        // Relative current speed
        auto relative_velocity = mybody->GetCurrentRelativeVelocity(NWU);
        relative_velocity.z() = 0.;         // keep only horizontal components
        auto vel2 = relative_velocity.Length2();

        // amplitude coefficient
        auto ak = 0.5*m_rho*vel2;

        force.x() = -0.07 * ak * m_transverse_area * cos(alpha);
        force.y() = 0.6 * ak * m_lateral_area * sin(alpha);

        auto m1 = std::min(0.4 * (1. - 2.*std::abs(alpha) / chrono::CH_C_PI), 0.25);
        auto m2 = std::max(m1, -0.2);
        moment.z() = force.y() * (m_xc + m2 * m_Lpp);

        // force in global reference frame
        force = mybody->Dir_Body2World(force);

    }



}