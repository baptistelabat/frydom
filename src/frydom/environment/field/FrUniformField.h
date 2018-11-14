//
// Created by camille on 27/02/18.
//

#ifndef FRYDOM_FRUNIFORMFIELD_H
#define FRYDOM_FRUNIFORMFIELD_H

#include "chrono/core/ChVector.h"  // TODO supprimer a terme

#include "frydom/core/FrObject.h"
#include "frydom/environment/field/FrFieldBase.h"

#include "frydom/core/FrVector.h"
#include "frydom/core/FrGeographic.h"
#include "frydom/core/FrUnits.h"

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


            chrono::ChVector<double> m_currentVector;  ///< The flux velocity vector of the current expressed in the NWU frame (NWU/GOTO)

            public:

            /// Default constructor: No current
            FrUniformCurrentField() {}

//            void Set() { m_currentVector = NORTH; }

            /// Constructor from a velocity vector embedding direction and magnitude (in m/s)
            void Set(chrono::ChVector<>  velocity_vector, FRAME_CONVENTION= NED, DIRECTION_CONVENTION = GOTO);

            /// Constructor from an angle and a magnitude.
            void Set(double  angle, double  magnitude,
                                  ANGLE_UNIT = DEG, SPEED_UNIT = KNOT, FRAME_CONVENTION= NED, DIRECTION_CONVENTION convention = GOTO);

            /// Constructor from a direction vector and a magnitude
            void Set(chrono::ChVector<>  unit_direction, double  magnitude,
                                  SPEED_UNIT = KNOT, FRAME_CONVENTION= NED, DIRECTION_CONVENTION convention = GOTO);


            // TODO: Ajouter les setters pour la direction et l'intensite
            void Set(const chrono::ChVector<>& unitDirection, double magnitude,
                     FRAME_CONVENTION frame=NED, DIRECTION_CONVENTION directionConvention=GOTO, SPEED_UNIT speedUnit=KNOT);

            void Update(double Time);

            chrono::ChVector<> GetFluxVector(FRAME_CONVENTION= NWU);

            chrono::ChVector<> GetComeFromVector(FRAME_CONVENTION= NWU);

            chrono::ChVector<> GetGoToVector(FRAME_CONVENTION= NWU);

            double GetAngle(DIRECTION_CONVENTION convention, FRAME_CONVENTION frame, ANGLE_UNIT = DEG);

            double GetMagnitude(SPEED_UNIT speedUnit=KNOT);

            double GetMagnitude2();

            void Initialize() {}

            void StepFinalize() {}

        };
















        ///////// REFACTORING ------------>>>>>>>>>>>>>>>>>>><


        class FrUniformField : public FrFieldBase {

            // TODO: Avoir un current asset sur le meme modele que FrForceAsset qui place un vecteur
            // courant devant le bateau avec la fleche sur un cercle entourant le bateau et pointant
            // vers le centre

            private:

                Velocity m_fluxVectorNWU = Velocity(0., 0., 0.);  ///< The flux velocity vector of the current expressed in the NWU frame (NWU/GOTO)

            public:

            /// Default constructor: No current
            FrUniformField() = default;

            /// Constructor from a velocity vector embedding direction and magnitude (in m/s)
            /// \param velocity
            /// \param fc
            /// \param dc
            void Set(const Velocity& velocity, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

            /// Constructor from an angle and a magnitude.
            /// \param angle
            /// \param magnitude
            /// \param angleUnit
            /// \param speedUnit
            /// \param fc
            /// \param dc
            void Set(double  angle, double  magnitude,
                     ANGLE_UNIT angleUnit, SPEED_UNIT speedUnit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);


            void Set(std::function<Velocity(FRAME_CONVENTION)> direction, double magnitude,
                     SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            ///
            /// \param magnitude
            /// \param speed_unit
            /// \param dc
            void SetNorth(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            ///
            /// \param magnitude
            /// \param speed_unit
            /// \param dc
            void SetNorthEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            ///
            /// \param magnitude
            /// \param speed_unit
            /// \param dc
            void SetEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            ///
            /// \param magnitude
            /// \param speed_unit
            /// \param dc
            void SetSouthEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            ///
            /// \param magnitude
            /// \param speed_unit
            /// \param dc
            void SetSouth(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            ///
            /// \param magnitude
            /// \param speed_unit
            /// \param dc
            void SetSouthWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            ///
            /// \param magnitude
            /// \param speed_unit
            /// \param dc
            void SetWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            ///
            /// \param magnitude
            /// \param speed_unit
            /// \param dc
            void SetNorthWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Return the flow velocity at a given point in world frame
            /// \param worldPos Position of the Point in world frame
            /// \param fc Frame convention (NED/NWU)
            /// \return Velocity in world frame
            Velocity GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const override;

            /// Update the state of the field model (virtual pure)
            /// \param time Current time of the simulation
            void Update(double time);

            /// Initialize the field object
            void Initialize() override;

            /// Method of be applied at the end of each time step
            void StepFinalize() override;

        };

}; // end namespace frydom

#endif //FRYDOM_FRUNIFORMFIELD_H
