// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef FRYDOM_FRFIELDBASE_H
#define FRYDOM_FRFIELDBASE_H

#include "frydom/core/common/FrObject.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"

namespace frydom {

    /**
     * \class FrFieldBase
     * \brief Class for defining in general the type of flow field used (uniform field, current or wind).
     */

    template <typename OffshoreSystemType>
    class FrFieldBase : public FrObject<OffshoreSystemType> {

    public:
        /// Default constructor
        FrFieldBase() = default;

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

    };
}

#endif //FRYDOM_FRFIELDBASE_H
