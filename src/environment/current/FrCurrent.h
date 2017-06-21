// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// Base for marine current modeling.
//
// =============================================================================

#ifndef FRYDOM_FRCURRENT_H
#define FRYDOM_FRCURRENT_H

//#include <cmath>

#define SQRT_2_2 sqrt(2.)/2.
#define KH 1000./3600.

// TODO: definir une classe de base pour le champ de courant et de vent (et de houle) afin de ne pas
// repliquer le code concernant la gestion des unites de vitesse, des conventions de direction ("vient de")
// et des reperes d'expression.

namespace frydom {
    namespace environment {

        // Forward declaration
//    class ChVector;


        class FrCurrent {

        public:

            // TODO: Cet enum aurait plus sa place directement dans le namespace frydom
            enum FrFrame { // TODO: Renommer en FrDefaultInertialFrame
                NWU,
                NED
            };
            // TODO: question : doit-on rendre le choix NED/NWU global (reference dans FrOffshoreSystem) ???

            // TODO: Cet enum aurait plus sa place directement dans le namespace frydom
            enum FrAngleUnit {
                DEG,
                RAD
            };
            // TODO: Cet enum aurait plus sa place directement dans le namespace frydom
            enum FrSpeedUnit {  // TODO: ajouter k/h ?
                MS,   ///< m/s
                KNOT  ///< nautical knot
            };

            // FIXME: inutile maintenant qu'on a les NORTH, EAST... -> seulement les methodes en ChVector udir
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

            /// Default constructor: No current
            FrCurrent();

            /// Constructor from a velocity vector embedding direction and strength
            FrCurrent(chrono::ChVector<> const velocity_vector, FrFrame= NED);

            /// Constructor from an angle and a strength
            FrCurrent(double const angle, double const velocity, FrAngleUnit= DEG, FrSpeedUnit= KNOT, FrFrame= NED);

            /// Constructor from a direction vector and a strength
            FrCurrent(chrono::ChVector<> const unit_direction, double const velocity, FrSpeedUnit= KNOT, FrFrame= NED);

            /// Constructor from a standard direction and a strength
            FrCurrent(FrDirectionQuadrant const quadrant, double const velocity, FrSpeedUnit= KNOT);

            /// Destrcutor
            ~FrCurrent() {}

            /// Get the strength of the current
            double getVelocity(FrSpeedUnit= MS);

            /// Set the strength of the current, keeping the current direction
            void setVelocity(double const velocity, FrSpeedUnit= KNOT);

            /// Get the direction of the current
            chrono::ChVector<> getDirection(FrFrame= NED);

            /// Set the current's direction given an angle
            void setDirection(double const angle, FrAngleUnit= DEG, FrFrame= NED);

            /// Set the current's direction given a unit direction vector (not checked !)
            void setDirection(chrono::ChVector<> const unit_vector, FrFrame= NED);

            /// Set the current's direction given a standard direction vector
            void setDirection(FrDirectionQuadrant const quadrant);

            /// Initialize the current field with a velocity vector embedding direction and strength
            void Initialize(chrono::ChVector<> const velocity_vector, FrFrame= NED);

            /// Initialize the current field with an angle and a velocity strength
            void
            Initialize(double const angle, double const velocity, FrAngleUnit= DEG, FrSpeedUnit= KNOT, FrFrame= NED);

            /// Initialize the current field with a unit vector direction and a velocity strength
            void
            Initialize(chrono::ChVector<> const unit_direction, double const velocity, FrSpeedUnit= KNOT, FrFrame= NED);

            /// Initialize the current field with a standard direction and a velocity strength
            void Initialize(FrDirectionQuadrant const quadrant, double const velocity, FrSpeedUnit= KNOT);

            /// Get the current velocity vector
            void get(chrono::ChVector<> &velocity_vector, FrFrame= NWU);

            /// Get the current unit direction and velocity of the current
            void get(chrono::ChVector<> &unit_vector, double &velocity, FrFrame= NWU, FrSpeedUnit= MS);


        private:
            // Definir cette fonction pratique directement dans le namespace environment !!
            // FIXME: encore utile ?
            chrono::ChVector<> quadrantToDir(FrDirectionQuadrant quadrant);

        };

        // SYMBOLIC DIRECTIONS EXPRESSED IN THE NED FRAME (please not forget the NED aspect !)
        extern const chrono::ChVector<double> NORTH;        ///< Current to the north
        extern const chrono::ChVector<double> NORTH_EAST;   ///< Current to the north/east
        extern const chrono::ChVector<double> EAST;         ///< Current to the east
        extern const chrono::ChVector<double> SOUTH_EAST;   ///< Current to the south/east
        extern const chrono::ChVector<double> SOUTH;        ///< Current to the south
        extern const chrono::ChVector<double> SOUTH_WEST;   ///< Current to the south/west
        extern const chrono::ChVector<double> WEST;         ///< Current to the west
        extern const chrono::ChVector<double> NORTH_WEST;   ///< Current to the north/west

    }  // end namespace environment
}  // end namespace frydom

#endif //FRYDOM_FRCURRENT_H
