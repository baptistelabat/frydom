// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRSTEADYHEAVEFORCE_H
#define FRYDOM_FRSTEADYHEAVEFORCE_H

#include "frydom/core/force/FrForce.h"

namespace frydom {

    /**
     * \class FrSteadyHeaveForce
     * \brief Class for computing a steady heave load.
     */
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
