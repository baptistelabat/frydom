//
// Created by frongere on 23/06/17.
//

#ifndef FRYDOM_CONSTANTS_H
#define FRYDOM_CONSTANTS_H

#include <cmath>
#include <iostream>
#include "chrono/core/ChVector.h"

// Forward declaration
namespace chrono {
    template <class Real>
    class ChVector;
}

namespace frydom {

    // =================================================================================================================
    // SYMBOLIC DIRECTIONS EXPRESSED IN THE NED FRAME (please do not forget the NED aspect !)
    // =================================================================================================================
    extern const chrono::ChVector<double> NORTH;        ///< North direction
    extern const chrono::ChVector<double> NORTH_EAST;   ///< North-East direction
    extern const chrono::ChVector<double> EAST;         ///< East direction
    extern const chrono::ChVector<double> SOUTH_EAST;   ///< South-East direction
    extern const chrono::ChVector<double> SOUTH;        ///< South direction
    extern const chrono::ChVector<double> SOUTH_WEST;   ///< South-West direction
    extern const chrono::ChVector<double> WEST;         ///< West direction
    extern const chrono::ChVector<double> NORTH_WEST;   ///< North-West direction

    // enum to specify between NWU and NED frames
    enum FrFrame {
        NWU,
        NED,
    };

    enum FrRefSyst {
        ABS,   ///< Absolute system of reference
        REL    ///< Relative system of reference
    };


    // TODO : placer les fonctions de conversion NED/NWU dans FrEulerAngles.h
    /// Transform either a NED vector into a NWU vector or a NWU vector into a NED vector (inline)
    template <class Real=double>
    inline chrono::ChVector<Real> swap_NED_NWU(chrono::ChVector<Real> const vect) {
        auto new_vect = vect;
        new_vect.y() = - new_vect.y();
        new_vect.z() = - new_vect.z();
        return new_vect;
    }

    /// Transform a NED vector into NWU
    template <class Real=double>
    inline chrono::ChVector<Real> NED2NWU(chrono::ChVector<Real> const vect) {
        return swap_NED_NWU(vect);
    }

    /// Transform a NWU vector into NED
    template <class Real=double>
    inline chrono::ChVector<Real> NWU2NED(chrono::ChVector<Real> const vect) {
        return swap_NED_NWU(vect);
    }


}  // end namespace frydom

#endif //FRYDOM_CONSTANTS_H
