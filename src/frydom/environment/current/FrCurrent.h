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
            chrono::ChVector<> m_current_vector;  ///< The flux velocity vector of the current expressed in the NWU frame (NWU/GOTO)

    public:

        /// Default constructor: No current
        FrCurrent() : m_current_vector(chrono::VNULL) {}

        /// Constructor from a velocity vector embedding direction and magnitude (in m/s)
        explicit FrCurrent(chrono::ChVector<>  velocity_vector,
                           FrFrame= NED, FrDirectionConvention= GOTO);

        /// Constructor from an angle and a magnitude.
        FrCurrent(double  angle, double  magnitude,
                  FrAngleUnit= DEG, FrSpeedUnit= KNOT, FrFrame= NED, FrDirectionConvention= GOTO);

        /// Constructor from a direction vector and a magnitude
        FrCurrent(chrono::ChVector<>  unit_direction, double  magnitude,
                  FrSpeedUnit= KNOT, FrFrame= NED, FrDirectionConvention= GOTO);

        void Update(double Time);

        chrono::ChVector<> GetFluxVector(FrFrame= NWU);

        chrono::ChVector<> GetComeFromVector(FrFrame= NWU);

        chrono::ChVector<> GetGoToVector(FrFrame= NWU);

        double GetAngle(FrDirectionConvention convention, FrFrame frame, FrAngleUnit= DEG);

        double GetMagnitude();

        double GetMagnitude2();



    };


}  // end namespace environment
}  // end namespace frydom

#endif //FRYDOM_FRCURRENT_H
