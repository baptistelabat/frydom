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


#ifndef FRYDOM_FRAIRYREGULARWAVEFIELD_H
#define FRYDOM_FRAIRYREGULARWAVEFIELD_H

#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"


namespace frydom {

    //Forward Declaration
    class FrFreeSurface_;

    /**
     * \class FrAiryRegularWaveField
     * \brief Class which deals with regular wave fields without optimization for a better parallelization.
     */
    class FrAiryRegularWaveField : public FrWaveField_ {
    protected:

        double m_height = 0.;   ///< Wave amplitude
        double m_period = 0.;   ///< Wave Period
        double m_omega = 0;     ///< Wave Frequency
        double m_k = 0.;        ///< Wave Number
        double m_dirAngle = 0.; ///< Wave direction
                                ///< used internally with the conventions : NWU, GOTO, and unit : RAD; [0,2PI]

        std::unique_ptr<FrKinematicStretching_> m_verticalFactor;    /// Vertical scale velocity factor with stretching

    public:

        /// Default constructor
        /// \param freeSurface pointer to the free surface, to which the wave field belongs
        explicit FrAiryRegularWaveField(FrFreeSurface_* freeSurface);

        /// Set the wave height of the regular Airy wave filed
        /// \param height wave height
        void SetWaveHeight(double height);

        /// Set the wave period of the regular Airy wave filed
        /// \param period wave period
        /// \param unit unit of the wave period, (seconds by default)
        void SetWavePeriod(double period, FREQUENCY_UNIT unit = S);

        /// Set the wave direction of the regular Airy wave filed, using angle, from North direction
        /// \param dirAngle wave direction
        /// \param unit wave direction unit (RAD/DEG)
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        void SetDirection(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        /// Set the wave direction of the regular Airy wave filed
        /// \param direction wave direction
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        void SetDirection(const Direction& direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        /// Set the stretching type, to avoid irregularities for quantities calculated above the free surface
        /// \param type stretching type (NO_STRETCHING/VERTICAL/EXTRAPOLATE/WHEELER/CHAKRABARTI/DELTA)
        void SetStretching(FrStretchingType type);

        /// Get the wave height of the regular Airy wave field
        /// \return wave height, in meters
        double GetWaveHeight() const;;

        /// Get the wave period of the regular Airy wave field
        /// \param unit wave period unit (seconds by default)
        /// \return wave period
        double GetWavePeriod(FREQUENCY_UNIT unit = S) const;;

        /// Get the wave direction angle, from North direction, of the regular Airy wave field
        /// \param unit angle unit
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        /// \return wave direction angle
        double GetDirectionAngle(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const;;

        /// Get the wave direction of the regular Airy wave field
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        /// \return wave direction
        Direction GetDirection(FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const;;

        /// Get the wave length of the regular Airy wave field
        /// \return wave length
        double GetWaveLength() const;

        //------------------------------------MAIN GETTERS----------------------------------//

        /// Get the complex wave elevation at the position (x,y,0), of the regular Airy wave field
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return complex wave elevation, in meters
        virtual std::vector<std::vector<Complex>> GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const;

        /// Return the complex eulerian fluid particule velocity in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param fc frame convention (NED/NWU)
        /// \return complex eulerian fluid particule velocity, in m/s
        virtual mathutils::Vector3d<Complex> GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const;

        /// Get the wave elevation at the position (x,y,0), of the regular Airy wave field
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return wave elevation, in meters
        double GetElevation(double x, double y, FRAME_CONVENTION fc) const final;

        /// Return the eulerian fluid particule velocity in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule velocity, in m/s
        Velocity GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const final;

        /// Return the eulerian fluid particule acceleration in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule acceleration, in m/s²
        Acceleration GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const final;

        // -------------------------------------- Wave characteristics -----------------------------

        std::vector<double> GetWaveFrequencies(FREQUENCY_UNIT unit) const override;

        std::vector<double> GetWaveNumbers() const override;

        std::vector<std::vector<double>> GetWaveAmplitudes() const override;

        std::vector<double> GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const override;

    };

}  // end namespace frydom

#endif //FRYDOM_FRAIRYREGULARWAVEFIELD_H
