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


#ifndef FRYDOM_FRSHIP_H
#define FRYDOM_FRSHIP_H

#include "FrHydroBody.h"

namespace frydom {

    // Forward declaration

    /**
     * \class FrShip
     * \brief Class not used.
     */
    class FrShip : public FrHydroBody {
        
    private:

    public:
		FrShip() : FrHydroBody() {}

        ~FrShip() {}

        /// Update subpart of the ship and then update hydrodynamics data
        void Update(bool update_assets = true) override;

        void StepFinalize() override;

        const double GetHeadingVelocity(FRAME_CONVENTION frame) const {
            return GetAngularVelocity(frame)[2];
        }

	};


}  // end namespace frydom

#endif //FRYDOM_FRSHIP_H
