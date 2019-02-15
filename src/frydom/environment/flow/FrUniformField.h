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


#ifndef FRYDOM_FRUNIFORMFIELD_H
#define FRYDOM_FRUNIFORMFIELD_H

#include "chrono/core/ChVector.h"  // TODO supprimer a terme

#include "frydom/core/common/FrObject.h"
#include "FrFieldBase.h"

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrUnits.h"

using namespace mathutils;

namespace frydom {

        /**
        * \class FrUniformCurrentField
        * \brief Class for defining a uniform current.
        */
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
















        // REFACTORING ------------>>>>>>>>>>>>>>>>>>><

        /**
        * \class FrUniformField
        * \brief Class for defining a uniform field (current or wind).
        */
        class FrUniformField : public FrFieldBase {

            // TODO: Avoir un current asset sur le meme modele que FrForceAsset qui place un vecteur
            // courant devant le bateau avec la fleche sur un cercle entourant le bateau et pointant
            // vers le centre

            private:

                Velocity m_fluxVectorNWU = Velocity(0., 0., 0.);  ///< The flux velocity vector of the current expressed in the NWU frame (NWU/GOTO)

            public:

            /// Default constructor: No field
            FrUniformField() = default;

            /// Definition of the uniform field from flux vector
            /// \param velocity Flux vector of the field
            /// \param fc Frame convention (NED/NWU)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void Set(const Velocity& velocity, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field from angle and magnitude
            /// \param angle Direction angle of the flow
            /// \param magnitude Velocity speed of the flow
            /// \param angleUnit Angle unit (RAD/DEG)
            /// \param speedUnit Speed unit (MS/KMH/KNOT)
            /// \param fc Frame convention (NED/NWU)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void Set(double  angle, double  magnitude,
                     ANGLE_UNIT angleUnit, SPEED_UNIT speedUnit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field from direction and magnitude
            /// \param direction Direction
            /// \param magnitude Velocity speed of the flow
            /// \param speed_unit Speed unit (MS/KMH/KNOT)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void Set(std::function<Velocity(FRAME_CONVENTION)> direction, double magnitude,
                     SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field aligned with north
            /// \param magnitude Velocity speed of the flow
            /// \param speed_unit Speed unit (MS/KMH/KNOT)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void SetNorth(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field aligned with north-east
            /// \param magnitude Velocity speed of the flow
            /// \param speed_unit Speed unit (MS/KMH/KNOT)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void SetNorthEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field aligned with east
            /// \param magnitude Velocity speed of the flow
            /// \param speed_unit Speed unit (MS/KMH/KNOT)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void SetEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field aligned with south-east
            /// \param magnitude Velocity speed of the flow
            /// \param speed_unit Speed unit (MS/KMH/KNOT)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void SetSouthEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field aligned with south
            /// \param magnitude Velocity speed of the flow
            /// \param speed_unit Speed unit (MS/KMH/KNOT)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void SetSouth(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field aligned with south-west
            /// \param magnitude Velocity speed of the flow
            /// \param speed_unit Speed unit (MS/KMH/KNOT)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void SetSouthWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field aligned with west
            /// \param magnitude Velocity speed of the flow
            /// \param speed_unit Speed unit (MS/KMH/KNOT)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void SetWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Definition of the uniform field aligned with north-west
            /// \param magnitude Velocity speed of the flow
            /// \param speed_unit Speed unit (MS/KMH/KNOT)
            /// \param dc Direction convention (GOTO/COMEFROM)
            void SetNorthWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc);

            /// Return the flux vector at a given point
            /// \param worldPos Position of point in world frame
            /// \param fc Frame convention (NED/NWU)
            /// \return Flux Vector
            Velocity GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const override;

            /// Update the state of the field model
            /// \param time Current time of the simulation
            void Update(double time);

            /// Initialize the field object
            void Initialize() override;

            /// Method of be applied at the end of each time step
            void StepFinalize() override;

        };

}; // end namespace frydom

#endif //FRYDOM_FRUNIFORMFIELD_H
