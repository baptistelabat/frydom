//
// Created by frongere on 23/06/17.
//

#ifndef FRYDOM_CONSTANTS_H
#define FRYDOM_CONSTANTS_H

#include <cmath>
#include <iostream>

#include "frydom/core/common/FrException.h"

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


    inline FRAME_CONVENTION STRING2FRAME(const std::string& field) {

        if (field == "NED") {
            return NED;
        } else if (field == "NWU") {
            return NWU;
        } else {
            throw FrException("unknown value for the direction convention");
        }

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


    inline DIRECTION_CONVENTION STRING2DIRECTION(const std::string& field) {
        if (field == "GOTO") {
            return GOTO;
        } else if (field == "COMEFROM") {
            return COMEFROM;
        } else {
            throw FrException("unknown value for the direction convention");
        }

    }


}  // end namespace frydom

#endif //FRYDOM_CONSTANTS_H
