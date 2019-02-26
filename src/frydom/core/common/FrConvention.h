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


#ifndef FRYDOM_CONSTANTS_H
#define FRYDOM_CONSTANTS_H


#include <string>
#include "frydom/core/common/FrException.h"


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
