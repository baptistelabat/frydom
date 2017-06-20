//
// Created by frongere on 20/06/17.
//

#ifndef FRYDOM_FRCURRENT_H
#define FRYDOM_FRCURRENT_H

#include <cmath>

#define SQRT_2_2 sqrt(2.)/2.

// TODO: definir une classe de base pour le champ de courant et de vent (et de houle) afin de ne pas
// repliquer le code concernant la gestion des unites de vitesse, des conventions de direction ("vient de")
// et des reperes d'expression.

namespace frydom {
    namespace environment {

        // Forward declaration
//    class ChVector;


        class FrCurrent {

        public:
            enum FrFrame {
                NWU,
                NED
            };

            enum FrAngleUnit {
                DEG,
                RAD
            };

            enum FrSpeedUnit {
                MS,   ///< m/s
                KNOT  ///< nautical knot
            };

            enum FrDirectionQuadrant {
                N,
                NE,
                E,
                SE,
                S,
                SW,
                W,
                NW,
            };

        private:
            // FIXME: ce vecteur doit representer le flux. Par contre, les informations d'angle sont courant porte vers et non vient de
            // FIXME: Corriger les information d'angle qui ne sont pas consistantes.
            chrono::ChVector<> m_velocity_vector;  ///< the velocity vector as seen by a body in the flux, expressed in the e frame

        public:

            FrCurrent();

            FrCurrent(chrono::ChVector<> const velocity_vector, FrFrame= NED);

            FrCurrent(double const angle, double const velocity, FrAngleUnit= DEG, FrSpeedUnit= KNOT, FrFrame= NED);

            FrCurrent(chrono::ChVector<> const unit_direction, double const velocity, FrSpeedUnit= KNOT, FrFrame= NED);

            FrCurrent(FrDirectionQuadrant const quadrant, double const velocity, FrSpeedUnit= KNOT);

            ~FrCurrent() {}

            double getVelocity(FrSpeedUnit= MS);

            void setVelocity(double const velocity, FrSpeedUnit= KNOT);

            chrono::ChVector<> getDirection(FrFrame= NED);

            void setDirection(double const angle, FrAngleUnit= DEG, FrFrame= NED);

            void setDirection(chrono::ChVector<> const unit_vector, FrFrame= NED);

            void setDirection(FrDirectionQuadrant const quadrant);

            void Initialize(chrono::ChVector<> const velocity_vector, FrFrame= NED);

            void
            Initialize(double const angle, double const velocity, FrAngleUnit= DEG, FrSpeedUnit= KNOT, FrFrame= NED);

            void
            Initialize(chrono::ChVector<> const unit_direction, double const velocity, FrSpeedUnit= KNOT, FrFrame= NED);

            void Initialize(FrDirectionQuadrant const quadrant, double const velocity, FrSpeedUnit= KNOT);

            void get(chrono::ChVector<> &velocity_vector, FrFrame= NWU);

            void get(chrono::ChVector<> &unit_vector, double &velocity, FrFrame= NWU, FrSpeedUnit= MS);


        private:

            chrono::ChVector<> quadrantToDir(FrDirectionQuadrant quadrant);

        };


        // CONSTANTS DIRECTIONS EXPRESSED IN THE NWU FRAME
        extern const chrono::ChVector<double> NORTH;
        extern const chrono::ChVector<double> NORTH_EAST;
        extern const chrono::ChVector<double> EAST;
        extern const chrono::ChVector<double> SOUTH_EAST;
        extern const chrono::ChVector<double> SOUTH;
        extern const chrono::ChVector<double> SOUTH_WEST;
        extern const chrono::ChVector<double> WEST;
        extern const chrono::ChVector<double> NORTH_WEST;


    }  // end namespace environment
}  // end namespace frydom

#endif //FRYDOM_FRCURRENT_H
