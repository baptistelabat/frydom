//
// Created by Lucas Letournel on 12/09/18.
//

#include <frydom/core/FrHydroBody.h>
#include "FrQuadraticDamping.h"

#include "chrono/physics/ChBody.h"

#include "cmath"

namespace frydom {


    void FrQuadraticDamping::UpdateState() {
        // FIXME: Il faut prendre en compte les vitesses relatives du fluide sur la carene et pas la vitesse absolue du corps
        // pour appliquer l'amortissement !! Sinon, on est over-damped !!!!
        // FIXME : a changer absolument et voir le modele de force de courant pour le calcul de la vitesse relative

        // Absolute linear velocity
        auto linear_vel = Body->GetPos_dt();
        auto angularVelocity = Body->GetWvel_par();

        force.x() = m_projectedSection.x()*m_translationalDampings.x() * linear_vel.x() * abs(linear_vel.x());
        force.y() = m_projectedSection.y()*m_translationalDampings.y() * linear_vel.y() * abs(linear_vel.y());
        force.z() = m_projectedSection.z()*m_translationalDampings.z() * linear_vel.z() * abs(linear_vel.z());

        auto rho = dynamic_cast<FrHydroBody*>(Body)->GetSystem()->GetEnvironment()->GetWaterDensity();
        force = -0.5*rho*force;

//        moment.x() = - m_seakeepingDampings.y() * angularVelocity.x();
//        moment.y() = - m_seakeepingDampings.z() * angularVelocity.y();
//        moment.z() = - m_maneuveuringDampings.z() * angularVelocity.z();
//        moment = Body->Dir_World2Body(moment);

    }

    void FrQuadraticDamping::Initialize() {
        assert(!(m_projectedSection.x()==0));
        assert(!(m_projectedSection.y()==0));
        assert(!(m_projectedSection.z()==0));
        FrForce::Initialize();

    }


}  // end namespace frydom