//
// Created by camille on 13/11/18.
//

#ifndef FRYDOM_FRFIELDBASE_H
#define FRYDOM_FRFIELDBASE_H

#include "frydom/core/FrObject.h"
#include "frydom/core/FrVector.h"

namespace frydom {

    class FrFieldBase : public FrObject {

    public:
        /// Default constructor
        FrFieldBase() {}

        /// Return the flow velocity at a given point in world frame
        /// \param worldPos Position of the Point in world frame
        /// \param fc Frame convention (NED/NWU)
        /// \return Velocity in world frame
        virtual Velocity GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const = 0;

        /// Update the state of the field model (virtual pure)
        /// \param time Current time of the simulation
        virtual void Update(double time) = 0;

        /// Initialize the field object
        void Initialize() override {};

        /// Method of be applied at the end of each time step
        void StepFinalize() override {};

    };
}

#endif //FRYDOM_FRFIELDBASE_H
