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

        // 1- Ship's velocity vector expressed in NED frame
        auto body_velocity = NWU2NED(GetBody()->GetPos_dt());

        // 2- Current vector in the NED frame
        auto current_velocity = GetCurrent()->GetVelocityVector(NED);

        // 3- Relative velocity
        auto relative_velocity = body_velocity - current_velocity;

        // 4- Direction
        auto alpha = degrees(atan2(relative_velocity.y(), relative_velocity.x()));

        // FIXME: TERMINER



    }

}  // end namespace environment
}  // end namespace frydom
