//
// Created by camille on 27/02/18.
//

#ifndef FRYDOM_FRUNIFORMCURRENTFIELD_H
#define FRYDOM_FRUNIFORMCURRENTFIELD_H

#include "chrono/core/ChVector.h"

#include "frydom/core/FrObject.h"
#include "frydom/core/FrGeographic.h"
#include "frydom/environment/FrConventions.h"
#include "MathUtils/MathUtils.h"

using namespace mathutils;

namespace frydom {

        class FrUniformCurrentField : virtual public FrObject,
                                      public std::enable_shared_from_this<FrUniformCurrentField> {

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

            // TODO: separer l'intensite et la direction et avoir le m_currentVector en cache...


            chrono::ChVector<> m_currentVector = NORTH;  ///< The flux velocity vector of the current expressed in the NWU frame (NWU/GOTO)

            public:

            /// Default constructor: No current
            FrUniformCurrentField() : m_currentVector(NORTH) {}

            void Set() { m_currentVector = NORTH; }

            /// Constructor from a velocity vector embedding direction and magnitude (in m/s)
            void Set(chrono::ChVector<>  velocity_vector,
                                           FRAME_CONVENTION= NED, FrDirectionConvention= GOTO);

            /// Constructor from an angle and a magnitude.
            void Set(double  angle, double  magnitude,
                                  ANGLE_UNIT = DEG, SPEED_UNIT = KNOT, FRAME_CONVENTION= NED, FrDirectionConvention convention = GOTO);

            /// Constructor from a direction vector and a magnitude
            void Set(chrono::ChVector<>  unit_direction, double  magnitude,
                                  SPEED_UNIT = KNOT, FRAME_CONVENTION= NED, FrDirectionConvention convention = GOTO);


            // TODO: Ajouter les setters pour la direction et l'intensite
            void Set(const chrono::ChVector<>& unitDirection, double magnitude,
                     FRAME_CONVENTION frame=NED, FrDirectionConvention directionConvention=GOTO, SPEED_UNIT speedUnit=KNOT);

            void Update(double Time);

            chrono::ChVector<> GetFluxVector(FRAME_CONVENTION= NWU);

            chrono::ChVector<> GetComeFromVector(FRAME_CONVENTION= NWU);

            chrono::ChVector<> GetGoToVector(FRAME_CONVENTION= NWU);

            double GetAngle(FrDirectionConvention convention, FRAME_CONVENTION frame, ANGLE_UNIT = DEG);

            double GetMagnitude(SPEED_UNIT speedUnit=KNOT);

            double GetMagnitude2();

            void Initialize() {}

            void StepFinalize() {}

        };

}; // end namespace frydom

#endif //FRYDOM_FRUNIFORMCURRENTFIELD_H
