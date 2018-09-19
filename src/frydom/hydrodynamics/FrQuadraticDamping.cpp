//
// Created by Lucas Letournel on 12/09/18.
//

#include <frydom/core/FrHydroBody.h>
#include "FrQuadraticDamping.h"

#include "chrono/physics/ChBody.h"

#include <cmath>

#include "frydom/environment/FrEnvironment.h"

namespace frydom {


    void FrQuadraticDamping::UpdateState() {

        /// Body linear velocity expressed in local (body) frame, relatively or not to the current velocity.
        chrono::ChVector<double> linear_vel;
        if (m_relative2Current)
            linear_vel = dynamic_cast<FrHydroBody*>(Body)->GetCurrentRelativeVelocity(NWU,LOCAL);
        else
            linear_vel = Body->TransformDirectionParentToLocal(Body->GetPos_dt());

        /// for each component :
        force.x() = m_Su * m_Cu * linear_vel.x() * abs(linear_vel.x());
        force.y() = m_Sv * m_Cv * linear_vel.y() * abs(linear_vel.y());
        force.z() = m_Sw * m_Cw * linear_vel.z() * abs(linear_vel.z());

        /// All components are multiplied by constant : 1/2*rho
        auto rho = dynamic_cast<FrOffshoreSystem*>(Body->GetSystem())->GetEnvironment()->GetWaterDensity();
        force = -0.5*rho*force;

        /// Resulting force is transformed back to global frame
        force = Body->TransformDirectionLocalToParent(force);
    }

    void FrQuadraticDamping::Initialize() {

        assert(!(m_relative2Current && (dynamic_cast<FrHydroBody*>(Body)== nullptr)));
        /// Check that if a damping coefficient is not null, the related section is also not null.
        assert(!(m_Su==0 && m_Cu!=0));
        assert(!(m_Sv==0 && m_Cv!=0));
        assert(!(m_Sw==0 && m_Cw!=0));
        FrForce::Initialize();

    }

    void FrQuadraticDamping::SetRelative2Current(bool relativeVelocity) {
        m_relative2Current = relativeVelocity;
    }


}  // end namespace frydom