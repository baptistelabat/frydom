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


#ifndef FRYDOM_FRFIELDBASE_H
#define FRYDOM_FRFIELDBASE_H

#include "frydom/core/common/FrObject.h"
#include "frydom/core/math/FrVector.h"

namespace frydom {

    /**
     * \class FrFieldBase
     * \brief Class for defining in general the type of flow field used (uniform field, current or wind).
     */
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
