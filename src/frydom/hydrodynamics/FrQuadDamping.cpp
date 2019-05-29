//
// Created by Lucas Letournel on 11/09/18.
//

#include "FrQuadDamping.h"
#include "chrono/physics/ChBody.h"

namespace frydom {
    void FrQuadDamping::UpdateState() {
        // FIXME: Il faut prendre en compte les vitesses relatives du fluide sur la carene et pas la vitesse absolue du corps
        // pour appliquer l'amortissement !! Sinon, on est over-damped !!!!
        // FIXME : a changer absolument et voir le modele de force de courant pour le calcul de la vitesse relative

        // Absolute linear velocity
        auto linear_vel = Body->GetPos_dt();
        auto angularVelocity = Body->GetWvel_par();

        force = - m_translationDamping * linear_vel.Length() * linear_vel;

//        force.x() = -m_maneuveuringDampings.x() * linear_vel.x();
//        force.y() = -m_maneuveuringDampings.y() * linear_vel.y();
//        force.z() = -m_seakeepingDampings.x() * linear_vel.z() * 0;

//        moment.x() = - m_seakeepingDampings.y() * angularVelocity.x();
//        moment.y() = - m_seakeepingDampings.z() * angularVelocity.y();
//        moment.z() = - m_maneuveuringDampings.z() * angularVelocity.z();
//        moment = Body->Dir_World2Body(moment);
    }
}