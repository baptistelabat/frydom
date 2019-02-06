//
// Created by camille on 01/08/18.
//

#ifndef FRYDOM_FRSTEADYHEAVEFORCE_H
#define FRYDOM_FRSTEADYHEAVEFORCE_H

#include "frydom/core/force/FrForce.h"

// TODO : plus utilisée dans le refactoring à supprimer

namespace frydom {

    class FrSteadyHeaveForce : public FrForce {

    private:
    public:
        void UpdateState() override;

    };



    void FrSteadyHeaveForce::UpdateState() {


        auto vx = Body->GetPos_dt().x();
        double forceZ;

        forceZ = -12.32426 * std::pow(vx, 3)
                - 2.8696 * std::pow(vx, 2);

        force.z() = forceZ;

        // ##CC monitoring force heave
        //std::cout << " ##### STEADY HEAVE FORCE : " << forceZ << std::endl;
        // ##CC
    }

}

#endif //FRYDOM_FRSTEADYHEAVEFORCE_H
