//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRBODY_H
#define FRYDOM_FRBODY_H


#include <chrono/physics/ChBodyAuxRef.h>
#include "frydom/core/FrConstants.h"

namespace frydom {
    class FrBody : public chrono::ChBodyAuxRef,
                   public std::enable_shared_from_this<FrBody> {

    public:
        /// Get the body absolute position (this of its reference point)
        chrono::ChVector<> GetPosition(FrFrame frame = NWU) {
            switch (frame) {
                case NWU:
                    return GetPos();
                case NED:
                    return NWU2NED(GetPos());
            }
        }

        /// Get the body orientation
        chrono::ChVector<> GetOrientation(FrFrame frame= NWU) {
            // TODO
        }

        /// Get the body velocity
        chrono::ChVector<> GetVelocity(FrFrame frame= NWU) {
            switch (frame) {
                case NWU:
                    return GetPos_dt();
                case NED:
                    return NWU2NED(GetPos_dt());
            }
        }

        /// Get the body angular velocity
        chrono::ChVector<> GetAngularVelocity(FrFrame frame= NWU) {
            //TODO
        }

    };

}  // end namespace frydom

#endif //FRYDOM_FRBODY_H
