//
// Created by Lucas Letournel on 21/11/18.
//

#ifndef FRYDOM_FRAIRYREGULARWAVEFIELD_H
#define FRYDOM_FRAIRYREGULARWAVEFIELD_H

#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"


namespace frydom {

    //Forward Declaration
    class FrFreeSurface_;


    class FrAiryRegularWaveField : public FrWaveField_ {
    private:
        /// Wave Height
        double m_height = 0.;
        /// Wave Period
        double m_period = 0.;
        /// Wave Frequency
        double m_omega = 0;
        /// Wave Number
        double m_k = 0.;
        /// Wave direction
        double m_dirAngle = 0.; // used internally with the conventions : NWU, GOTO, and unit : RAD; [0,2PI]
//        Direction m_direction = NORTH(NWU);

        /// Vertical scale velocity factor with stretching
        std::unique_ptr<FrKinematicStretching_> m_verticalFactor;

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

        /// Get the wave elevation at the position (x,y,0), of the regular Airy wave field
        /// \param x x position
        /// \param y y position
        /// \return wave elevation, in meters
        double GetElevation(double x, double y) const final;

        /// Get the complex wave elevation at the position (x,y,0), of the regular Airy wave field
        /// \param x x position
        /// \param y y position
        /// \return complex wave elevation, in meters
        Complex GetComplexElevation(double x, double y) const;

        /// Return the eulerian fluid particule velocity in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \return eulerian fluid particule velocity, in m/s
        Velocity GetVelocity(double x, double y, double z) const final;

        /// Return the complex eulerian fluid particule velocity in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \return complex eulerian fluid particule velocity, in m/s
        mathutils::Vector3d<Complex> GetComplexVelocity(double x, double y, double z) const;

        /// Return the eulerian fluid particule acceleration in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \return eulerian fluid particule acceleration, in m/s²
        Acceleration GetAcceleration(double x, double y, double z) const final;

        void Update(double time) final;;
    };
}

#endif //FRYDOM_FRAIRYREGULARWAVEFIELD_H
