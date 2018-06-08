//
// Created by camille on 05/06/18.
//

#ifndef FRYDOM_FRNODEDYNAMIC_H
#define FRYDOM_FRNODEDYNAMIC_H

#include "frydom/core/FrHydroBody.h"
#include "frydom/core/FrNode.h"
#include "chrono/physics/ChLinkMate.h"

namespace frydom {

    class FrNodeDynamic :   public FrHydroBody {

    private:
        std::shared_ptr<FrForce> m_force;

    public:
        /// Default constructor
        FrNodeDynamic();

        /// Define the motion of the node from the mass spring system with damping
        void SetSpringDamping(chrono::ChFrameMoving<>* ref_node,
                        const double T0=60.,
                        const double psi = 0.5);

        /// Define the motion of the node with steady velocity
        void SetSteadyMotion(chrono::ChVector<double> velocity);


    };


}

#endif //FRYDOM_FRNODEDYNAMIC_H
