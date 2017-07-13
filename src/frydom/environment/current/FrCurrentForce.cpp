//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBody.h"

#include <frydom/core/FrConstants.h>
#include <frydom/core/FrHydroBody.h>
#include "FrCurrentForce.h"

#include "frydom/IO/FrLoader.h"

//#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {
namespace environment {

    FrCurrentForce::FrCurrentForce(std::string yaml_file) {
        coeffs_table = std::move(IO::MakeCurrentPolarCoeffTable(yaml_file));
    }


//    environment::FrCurrent *FrCurrentForce::GetCurrent() {
//        auto system = dynamic_cast<FrOffshoreSystem *>(GetBody()->GetSystem());  // Downcasting of a ChSystem
//        return system->GetCurrent();
//    }

    void FrCurrentForce::UpdateState() {

        auto coucou = coeffs_table.Eval_cx(0.50);

        // TODO: plutot travailler dans NED puis ensuite passer en NWU pour les forces...

        // 1- Ship's velocity vector expressed in NED frame
        auto body = dynamic_cast<FrHydroBody*> (GetBody());
        auto current_flow = body->GetCurrentFlow();

//        auto body_velocity = NWU2NED(GetBody()->GetPos_dt());

        // TODO: c'est hydrobody qui doit avoir la capacite intrinseque de calculer le vecteur flux de courant relatif...
        // Meme chose pour le vent

        // 2- Current vector in the NED frame
//        auto current_velocity = GetCurrent()->GetVelocityVector(NED);

        // 3- Relative velocity
//        auto relative_velocity = body_velocity - current_velocity;

        // 4- Direction
//        auto alpha = degrees(atan2(relative_velocity.y(), relative_velocity.x()));

        // FIXME: TERMINER



    }



    }  // end namespace environment
}  // end namespace frydom
