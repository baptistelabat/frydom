//
// Created by frongere on 11/09/17.
//

#include "FrLinearDamping.h"
#include "chrono/physics/ChBody.h"

namespace frydom {


    void FrLinearDamping::UpdateState() {
        // FIXME: Il faut prendre en compte les vitesses relatives du fluide sur la carene et pas la vitesse absolue du corps
        // pour appliquer l'amortissement !! Sinon, on est over-damped !!!!
        // FIXME : a changer absolument et voir le modele de force de courant pour le calcul de la vitesse relative

        auto generalizedVelocity = 


        // Absolute linear velocity
        auto linear_vel = Body->GetPos_dt();
        auto angularVelocity = Body->GetWvel_par();

        force.x() = - m_translationalDampings.x() * linear_vel.x();
        force.y() = - m_translationalDampings.y() * linear_vel.y();
        force.z() = - m_translationalDampings.z() * linear_vel.z();

//        moment.x() = - m_seakeepingDampings.y() * angularVelocity.x();
//        moment.y() = - m_seakeepingDampings.z() * angularVelocity.y();
//        moment.z() = - m_maneuveuringDampings.z() * angularVelocity.z();
//        moment = Body->Dir_World2Body(moment);

    }


}  // end namespace frydom