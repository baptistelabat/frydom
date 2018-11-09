//
// Created by frongere on 23/06/17.
//

#ifndef FRYDOM_CONSTANTS_H
#define FRYDOM_CONSTANTS_H

#include <cmath>
#include <iostream>

// TODO : placer ici les services de geographie et l'inclusion de GeographicLib


// Forward declaration
namespace chrono {
    template <class Real>
    class ChVector;
}

namespace frydom {

    /// Absolute Reference frame conventions
    enum FRAME_CONVENTION {
        NWU,
        NED,
    };

    inline bool IsNWU(FRAME_CONVENTION fc) {
        return (fc == NWU);
    }

    inline bool IsNED(FRAME_CONVENTION fc) {
        return (fc == NED);
    }

    enum FrRefSyst {  // TODO : a retirer et n'utiliser que FRAME_CONVENTION
        LOCAL,
        PARENT,
    };


    enum DIRECTION_CONVENTION {
        GOTO,
        COMEFROM
    };

    inline bool IsGOTO(DIRECTION_CONVENTION dc) {
        return (dc == GOTO);
    }

    inline bool IsCOMEFROM(DIRECTION_CONVENTION dc) {
        return (dc == COMEFROM);
    }

}  // end namespace frydom

#endif //FRYDOM_CONSTANTS_H
