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

#ifndef FRYDOM_FRWAVEFIELD_H
#define FRYDOM_FRWAVEFIELD_H


#include "frydom/core/common/FrObject.h"
#include "frydom/core/math/FrVector.h"

#include "chrono/core/ChVector.h"

#include "FrWaveSpectrum.h"
#include "FrKinematicStretching.h"

#include "frydom/core/math/FrComplex.h"


namespace frydom {

    // Forward declarations
    class FrFreeSurface_;
    class FrLinearRampFunction_;

    /**
     * \class FrWaveField_
     * \brief Class for defining a wave field (null or linear).
     */
    class FrWaveField_ : public FrObject {

    public:

        enum WAVE_MODEL {
            NO_WAVES,
            LINEAR_WAVES
        };


    protected:

        FrFreeSurface_* m_freeSurface;        ///< Pointer to the free surface containing this wave field
        WAVE_MODEL m_waveModel = NO_WAVES;    ///< wave model (NO_WAVES, LINEAR_WAVES)

        // Cache attributes
        double c_ramp = 1.;                   ///< cache value of the time ramp applied on the wave field
        double c_time;                        ///< cache value of the time of the simulation
        double c_depth;                       ///< cache value of the depth. (depth = bathymetry + tidal)
        bool m_infinite_depth = false;        ///< Infinite depth boolean (if true, water depth is considered as infinite)

    public:

        /// Default Constructor
        /// \param freeSurface free surface containing this wave field
        explicit  FrWaveField_(FrFreeSurface_* freeSurface);

        /// Default destructor
        ~FrWaveField_() = default;

        /// Get the wave model
        /// \return wave model (NO_WAVES, LINEAR_WAVES)
        WAVE_MODEL GetWaveModel() const;

        //
        // Velocity & Acceleration
        //

        /// Get the wave elevation on the horizontal position (x,y)
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return wave elevation, in meters
        virtual double GetElevation(double x, double y, FRAME_CONVENTION fc) const = 0;

        /// Return the eulerian fluid particule velocity in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule velocity, in m/s
        virtual Velocity GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const = 0;

        /// Return the eulerian flow velocity. Return null vector if the point is upper the free surface elevation
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param cutoff if true, and z position above the wave elevation, return 0
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule velocity, in m/s
        virtual Velocity GetVelocity(double x, double y, double z, bool cutoff, FRAME_CONVENTION fc) const;


        /// Return the eulerian fluid particule velocity in global reference frame (from vector position)
        /// \param worldPos position
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule velocity, in m/s
        virtual Velocity GetVelocity(const Position& worldPos, FRAME_CONVENTION fc) const;

        /// Return the eulerian fluid particule acceleration in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule acceleration, in m/s²
        virtual Acceleration GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const = 0;

        /// Return the eulerian fluid particule acceleration in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param cutoff if true, and z position above the wave elevation, return 0
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule acceleration, in m/s²
        virtual Acceleration GetAcceleration(double x, double y, double z, bool cutoff, FRAME_CONVENTION fc) const;

        /// Return the eulerian fluid particule acceleration in global reference frame (from vector position)
        /// \param worldPos position
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule acceleration, in m/s²
        virtual Acceleration GetAcceleration(const Position& worldPos, FRAME_CONVENTION fc) const;

        /// Get the wave elevation for a set of point positions
        /// \param xVect x positions
        /// \param yVect y positions
        /// \param fc frame convention (NED/NWU)
        /// \return wave elevation, in meters
        virtual std::vector<std::vector<double>> GetElevation(const std::vector<double>& xVect,
                                                              const std::vector<double>& yVect, FRAME_CONVENTION fc) const;

        /// Return the eulerian fluid particule velocity in global reference frame (implemented in child)
        /// for a set of point positions
        /// \param xvect x positions
        /// \param yvect y positions
        /// \param zvect z positions
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule velocity, in m/s
        virtual std::vector<std::vector<std::vector<Velocity>>> GetVelocity(const std::vector<double>& xvect,
                                                                  const std::vector<double>& yvect,
                                                                  const std::vector<double>& zvect, FRAME_CONVENTION fc) const;
        //
        // Wave frequencies and direction
        //

        virtual std::vector<double> GetWaveFrequencies(FREQUENCY_UNIT unit) const = 0;

        virtual std::vector<double> GetWaveNumbers() const = 0;

        virtual std::vector<std::vector<double>> GetWaveAmplitudes() const = 0;

        virtual std::vector<double> GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const = 0;

        virtual std::vector<std::vector<Complex>> GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const = 0;

        //
        // Update
        //

        /// Initialize the state of the wave field
        void Initialize() override;

        /// Update the state of the free surface
        /// \param time time of the simulation
        virtual void Update(double time);

        /// Method called at the send of a time step.
        void StepFinalize() override;

    };


    /**
     * \class FrNullWaveField_
     * \brief Class for defining a null wave field.
     */
    class FrNullWaveField_ : public FrWaveField_ {

    public:
        explicit FrNullWaveField_(FrFreeSurface_* freeSurface);

        /// Get the wave elevation on the horizontal position (x,y)
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

        std::vector<double> GetWaveFrequencies(FREQUENCY_UNIT unit) const override;

        std::vector<double> GetWaveNumbers() const override;

        std::vector<std::vector<double>> GetWaveAmplitudes() const override;

        std::vector<double> GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const override;

        std::vector<std::vector<Complex>> GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const override;
    };

}  // end namespace frydom

#endif //FRYDOM_FRWAVEFIELD_H
