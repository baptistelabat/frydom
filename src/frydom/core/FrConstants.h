//
// Created by frongere on 23/06/17.
//

#ifndef FRYDOM_CONSTANTS_H
#define FRYDOM_CONSTANTS_H

#include <cmath>

// Forward declaration
namespace chrono {
    template <class Real>
    class ChVector;
}

namespace frydom {

    #define SQRT_2_2 sqrt(2.)/2.

    #define M_ONE_MILE 1852.                        ///> NUMBER OF METER IN ONE NAUTICAL MILE
    #define M_ONE_MINUTE 60.                        ///> NUMBER OF SECONDS IN ONE MINUTE
    #define M_ONE_HOUR (M_ONE_MINUTE*60.)           ///> NUMBER OF SECONDS IN ONE HOUR
    #define M_KNOT (M_ONE_MILE/M_ONE_HOUR)          ///> Conversion coeff knot -> m/s
    #define M_ONE_KM 1000.                          ///> ONE KILOMETER
    #define M_KMH (M_ONE_KM/M_ONE_HOUR)             ///> ONE KILOMETER BY HOUR

    #define M_PI_180 (M_PI/180.)                    ///> Conversion DEF->RAD
    #define M_DEG M_PI_180                          ///> Conversion DEG->RAD
    #define M_2PI (2.*M_PI)                         ///> 2*PI

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

    // enum
    enum FrAngleUnit {
        DEG,
        RAD
    };

    enum FrSpeedUnit {  // TODO: ajouter k/h ?
        MS,     ///> M/S
        KNOT,   ///> NAUTICAL KNOTS
        KMH     ///> KM/H
    };

    // =================================================================================================================
    // UTILITY FUNCTIONS
    // =================================================================================================================

    /// Convert nautical knots into m/s
    template <class T>
    inline T KNOT2MS(T velocity) {
        return velocity * M_KNOT;
    }

    /// Convert m/s into nautical knots
    template <class T>
    inline T MS2KNOT(T velocity) {
        return velocity / M_KNOT;
    }

    /// Convert km/h into m/s
    template <class T>
    inline T KMH2MS(T velocity) {
        return velocity * M_KMH;
    }

    /// Convert m/s into km/h
    template <class T>
    inline T MS2KMH(T velocity) {
        return velocity / M_KMH;
    }

    /// Convert km/h into nautical knots
    template <class T>
    inline T KMH2KNOT(T velocity) {
        return MS2KNOT(KMH2MS(velocity));
    }

    /// Convert nautical knots into km/h
    template <class T>
    inline T KNOT2KMH(T velocity) {
        return MS2KMH(KNOT2MS(velocity));
    }

    /// Converts a velocity from a unit to another
    template <class T>
    inline T convert_velocity_unit(const T velocity, FrSpeedUnit current_unit, FrSpeedUnit new_unit= MS) {

        T new_vel;
        // Expressing in M/S
        switch (current_unit) {
            case MS:
                new_vel = velocity;
                break;
            case KMH:
                new_vel = KMH2MS(velocity);
                break;
            case KNOT:
                new_vel = KNOT2MS(velocity);
        }

        // EXPRESSING IN NEW UNIT
        switch (new_unit) {
            case MS:
                return new_vel;
            case KMH:
                return MS2KMH(new_vel);
            case KNOT:
                return MS2KNOT(new_vel);
        }

    }


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

    // TODO: both two following functions should be realized as MACROS for performance
    /// CONVERSION DEG->RAD
    template <class T>
    inline T radians(const T a) {
        return a * M_PI_180;
    }

    /// CONVERSION RAD->DEG
    template <class T>
    inline T degrees(const T a) {
        return a / M_PI_180;
    }

    /// Reminder of mod(2*pi) to put back an angle expressed in radian into [0, 2pi[
    template <class Real=double>
    inline Real modulo2pi(const Real a) {
        return fmod(a, (Real)M_2PI);
    }

    /// Reminder of mod(360) to put back an angle expressed in degrees into [0, 360[
    template <class Real=double>
    inline Real modulo360(const Real a) {
        return fmod(a, (Real)360.);
    }

}  // end namespace frydom

#endif //FRYDOM_CONSTANTS_H
