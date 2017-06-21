//
// Created by frongere on 21/06/17.
//

#include "FrShip.h"
#include "../propeller/FrPropeller.h"

namespace frydom {


    void FrShip::AddPropeller(std::shared_ptr<FrPropeller> propeller) {

        // Adding propeller to the propeller list
        propellerlist.push_back(propeller);

        // Adding the propeller as a force in the force list
        AddForce(propeller);

    }

    void FrShip::RemovePropeller(std::shared_ptr<FrPropeller> propeller) {
        // Trying to remove objects not previously added?
        assert(std::find<std::vector<std::shared_ptr<FrPropeller>>::iterator>(propellerlist.begin(),
                                                                              propellerlist.end(),
                                                                              propeller) != propellerlist.end());

        // warning! linear time search
        propellerlist.erase(
                std::find<std::vector<std::shared_ptr<FrPropeller>>::iterator>(propellerlist.begin(),
                                                                               propellerlist.end(),
                                                                               propeller));

        RemoveForce(propeller);
    }

    chrono::ChVector<double> FrShip::GetShipVelocity() const {
        return GetCoord_dt().pos;
    }


}  // end namespace frydom