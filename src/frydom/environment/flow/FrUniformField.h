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

#include "FrFieldBase.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrUnits.h"


namespace frydom {

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

            /// Get the type name of this object
            /// \return type name of this object
            std::string GetTypeName() const override { return "UniformField"; }

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

        };


} // end namespace frydom

#endif //FRYDOM_FRUNIFORMFIELD_H
