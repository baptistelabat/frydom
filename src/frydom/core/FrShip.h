//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRSHIP_H
#define FRYDOM_FRSHIP_H

#include "FrHydroBody.h"

namespace frydom {

    // Forward declaration


    class FrShip : public FrHydroBody {


    private:

    public:
		FrShip() : FrHydroBody() {}

        ~FrShip() {}

        /// Update subpart of the ship and then update hydrodynamics data
        void Update(bool update_assets = true) override;

        void StepFinalize() override;

        const double GetHeadingVelocity(FrFrame frame) const {
            return GetAngularVelocity(frame)[2];
        }

	};


}  // end namespace frydom

#endif //FRYDOM_FRSHIP_H
