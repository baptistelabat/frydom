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

        // Absolute linear velocity
        auto linear_vel = Body->GetPos_dt();

        force.x() = - m_Dx * linear_vel.x();
        force.y() = - m_Dy * linear_vel.y();
        force.z() = 0.;

        auto moment_abs = chrono::ChVector<double>();
        moment.z() = - m_Dwz * Body->GetWvel_par().z();
    }


}  // end namespace frydom