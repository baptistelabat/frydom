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

#include "chrono/core/ChVector.h"
#include "frydom/core/FrConstants.h"
#include "frydom/environment/FrConventions.h"

// TODO: definir une classe de base pour le champ de courant et de vent (et de houle) afin de ne pas
// repliquer le code concernant la gestion des unites de vitesse, des conventions de direction ("vient de")
// et des reperes d'expression.

namespace frydom {
namespace environment {

    class FrCurrent {  // TODO: renommer en FrCurrentField

        // TODO: Avoir un current asset sur le meme modele que FrForceAsset qui place un vecteur
        // courant devant le bateau avec la fleche sur un cercle entourant le bateau et pointant
        // vers le centre

        // TODO: avoir les methodes:
        //  - GetFluxVector == GetGoToVector
        //  - GetComeFromVector
        //  - GetComeFromAngle
        //  - GetGoToAngle
        //  - GetMagnitude
        //  - GetMagnitude2


    private:
        // FIXME: ce vecteur doit representer le flux. Par contre, les informations d'angle sont courant porte vers et non vient de
        // FIXME: Corriger les information d'angle qui ne sont pas consistantes.
//            chrono::ChVector<> m_velocity_vector;  // On deprecie !! on stocke mainteant l'angle et la direction
        double angle;       ///> the current angle in a NED frame following the GOTO convention (in RAD...)
        double magnitude;   ///> the current magnitude in M/S

    public:

        /// Default constructor: No current
        FrCurrent();

        /// Constructor from an angle and a strength
        FrCurrent(double angle, double magnitude,
                  FrAngleUnit= DEG, FrSpeedUnit= KNOT, FrFrame= NED);

        /// Constructor from a velocity vector embedding direction and strength
        explicit FrCurrent(chrono::ChVector<> velocity_vector,
                           FrFrame= NED, FrSpeedUnit= MS);

        /// Constructor from a direction vector and a strength
        FrCurrent(chrono::ChVector<> unit_direction, double magnitude,
                  FrSpeedUnit= KNOT, FrFrame= NED);

//            /// Destructor
//            ~FrCurrent() = default;

        /// Get the strength of the current
        double getVelocity(FrSpeedUnit= MS);

        /// Set the strength of the current, keeping the current direction
        void setVelocity(double velocity, FrSpeedUnit= KNOT);

        /// Get the direction of the current
        chrono::ChVector<> getDirection(FrFrame= NED);

        /// Set the current's direction given an angle
        void setDirection(double angle, FrAngleUnit= DEG, FrFrame= NED);

        /// Set the current's direction given a unit direction vector (not checked !)
        void setDirection(chrono::ChVector<> unit_vector, FrFrame= NED);

        /// Initialize the current field with a velocity vector embedding direction and strength
        void Initialize(chrono::ChVector<> velocity_vector, FrFrame= NED);

        /// Initialize the current field with an angle and a velocity strength
        void
        Initialize(double angle, double velocity, FrAngleUnit= DEG, FrSpeedUnit= KNOT, FrFrame= NED);

        /// Initialize the current field with a unit vector direction and a velocity strength
        void
        Initialize(chrono::ChVector<> unit_direction, double velocity, FrSpeedUnit= KNOT, FrFrame= NED);

        /// Get the current velocity vector
        void get(chrono::ChVector<> &velocity_vector, FrFrame= NWU);

        /// Get the current unit direction and velocity of the current
        void get(chrono::ChVector<> &unit_vector, double &velocity, FrFrame= NWU, FrSpeedUnit= MS);

        chrono::ChVector<double> GetVelocityVector(FrFrame=NWU);
    };

}  // end namespace environment
}  // end namespace frydom

#endif //FRYDOM_FRCURRENT_H
