//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBody.h"

#include "FrCurrentForce.h"

#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {
namespace environment {

    environment::FrCurrent *FrCurrentForce::GetCurrent() {
        auto system = dynamic_cast<FrOffshoreSystem *>(GetBody()->GetSystem());  // Downcasting of a ChSystem
        return system->GetCurrent();
    }

    void FrCurrentForce::UpdateState() {

        // TODO: plutot travailler dans NED puis ensuite passer en NWU pour les forces...

        // 1- Recuperation du vecteur vitesse du bateau dans le repere NWU
        auto body_velocity = GetBody()->GetPos_dt();

//        std::cout << "VITESSE CORPS NED: " << body_velocity.x() << std::endl;

        // 2- Recuperation du vecteur vitesse du courant dans le repere NED
//        chrono::ChVector<double> current_velocity;
//        GetCurrent()->get(current_velocity, environment::FrCurrent::NWU);
        auto current_velocity = GetCurrent()->GetVelocityVector(NWU);

        // 3- Calcul de la vitesse relative
        auto relative_velocity = body_velocity - current_velocity;

        // 4- Computing the force



    }

}  // end namespace environment
}  // end namespace frydom
