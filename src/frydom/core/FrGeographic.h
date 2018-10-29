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

    namespace internal {

        // TODO : placer les fonctions de conversion NED/NWU dans FrEulerAngles.h
        /// Transform either a NED vector into a NWU vector or a NWU vector into a NED vector (inline)
//        template<class Real=double>
//        inline chrono::ChVector<Real> swap_NED_NWU(const chrono::ChVector<Real>& vect) {
//            auto new_vect = vect;
//            swap_NED_NWU(new_vect);
//            return new_vect;
//        }

//        template <class Real=double>
//        inline chrono::ChVector<Real>& swap_NED_NWU(chrono::ChVector<Real>& vect) {
//            vect.y() = -vect.y();
//            vect.z() = -vect.z();
//            return vect;
//        }

//        /// Transform a NED vector into NWU
//        template<class Real=double>
//        inline chrono::ChVector<Real> NED2NWU(const chrono::ChVector<Real>& vect) {
//            return swap_NED_NWU(vect);
//        }
//
//        /// Transform a NWU vector into NED
//        template<class Real=double>
//        inline chrono::ChVector<Real> NWU2NED(const chrono::ChVector<Real>& vect) {
//            return swap_NED_NWU(vect);
//        }
    }  // end namespace internal

}  // end namespace frydom

#endif //FRYDOM_CONSTANTS_H