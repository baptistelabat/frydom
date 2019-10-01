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


#ifndef FRYDOM_FRAIRYIRREGULARWAVEFIELD_H
#define FRYDOM_FRAIRYIRREGULARWAVEFIELD_H

#include <random>

#include "frydom/environment/ocean/freeSurface/waves/FrWaveSpectrum.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/FrKinematicStretching.h"


namespace frydom {

    //Forward Declaration
    class FrFreeSurface;
    class FrWaveSpectrum;
    class FrKinematicStretching;

    /**
     * \class FrAiryIrregularWaveField
     * \brief Class which deals with irregular wave field.
     */
    class FrAiryIrregularWaveField : public FrWaveField {
    protected:


        double m_minFreq = 0.;        ///< Minimum frequency
        double m_maxFreq = 0.;          ///< Maximum frequency
        unsigned int m_nbFreq = 40;     ///< Number of frequency to discretize

        double m_meanDir = 0;           ///< Mean wave direction
        unsigned int m_nbDir = 1;       ///< Number of directions to discretize

        std::unique_ptr<FrWaveSpectrum> m_waveSpectrum;    ///< Wave spectrum, by default JONSWAP (Hs=3m,Tp=9s,Gamma=3.3)

        std::vector<double> m_waveDirections;    ///< Wave directions vector
        std::vector<double> m_waveFrequencies;   ///< Wave frequencies vector
        std::vector<double> m_waveNumbers;       ///< Wave numbers vector

        std::vector<std::vector<double>> c_amplitude;    ///< cache value of the amplitude given by the wave spectrum
        std::unique_ptr<std::vector<std::vector<double>>> m_wavePhases;    ///< Table of wave phases,of dimensions (m_nbDir,m_nbFreq)
                                                                           ///< made unique to check at initialize() if wavePhases were given by the users,
                                                                           ///< or if they need to be randomly generated.
        std::unique_ptr<FrKinematicStretching> m_verticalFactor;    ///< Vertical scale velocity factor with stretching

    public:

        /// Default constructor
        /// \param freeSurface pointer to the free surface, to which the wave field belongs
        explicit FrAiryIrregularWaveField(FrFreeSurface* freeSurface);

        /// Set the wave frequencies, ie frequency discretization
        /// \param minFreq minimum frequency
        /// \param maxFreq maximum frequency
        /// \param nbFreq number of frequencies to discretize
        void SetWaveFrequencies(double minFreq, double maxFreq, unsigned int nbFreq);

        /// Set the mean wave direction angle
        /// \param dirAngle mean wave direction angle
        /// \param unit unit of the angle
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (COMEFROM/GOTO)
        void SetMeanWaveDirectionAngle(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        /// Set the mean wave direction
        /// \param direction mean wave direction
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (COMEFROM/GOTO)
        void SetMeanWaveDirection(Direction direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        /// Get the wave direction angle, from North direction, of the irregular Airy wave field
        /// \param unit angle unit
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        /// \return wave direction angle
        double GetMeanWaveDirectionAngle(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const;;

        /// Get the wave direction of the irregular Airy wave field
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        /// \return wave direction
        Direction GetMeanWaveDirection(FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const;;

        /// Get the wave phases for each couple of frequencies and directions
        /// \return wave phases
        std::vector<std::vector<double>>* GetWavePhases() const {return m_wavePhases.get();}

        /// Set the wave phases for each couple of frequencies and directions
        /// \param wavePhases wave phases
        void SetWavePhases(std::vector<std::vector<double>>& wavePhases);

        /// Set the parameters of the directional model
        /// \param nbDir direction discretization
        /// \param spreadingFactor spreading factor
        /// \param dirType directional model type (NONE, COS2S, TEST(for tests only))
        void SetDirectionalParameters(unsigned int nbDir, double spreadingFactor, WAVE_DIRECTIONAL_MODEL dirType=COS2S);

        /// Set the stretching type used to compute velocity and acceleration on positions above the free surface elevation
        /// \param type stretching type (NO_STRETCHING, VERTICAL, EXTRAPOLATE, WHEELER, CHAKRABARTI, DELTA)
        void SetStretching(STRETCHING_TYPE type);

//        void SetWaveSpectrum(WAVE_SPECTRUM_TYPE type);

        /// Set a Jonswap wave spectrum
        /// \param Hs significant height (meters)
        /// \param Tp peak period (seconds)
        /// \param gamma gamma factor of the Jonswap wave spectrum
        /// \return wave spectrum
        FrJonswapWaveSpectrum* SetJonswapWaveSpectrum(double Hs, double Tp, double gamma=3.3);

        /// Set a Pierson Moskowitz wave spectrum
        /// \param Hs significant height (meters)
        /// \param Tp peak period (seconds)
        /// \return wave spectrum
        FrPiersonMoskowitzWaveSpectrum* SetPiersonMoskovitzWaveSpectrum(double Hs, double Tp);

        /// Set a wave spectrum, based on the TEST wave spectrum type
        /// \return the TEST wave spectrum
        FrTestWaveSpectrum* SetTestWaveSpectrum();

        /// Get the wave spectrum
        /// \return wave spectrum
        FrWaveSpectrum* GetWaveSpectrum() const;

        ///Generate random wave phases
        void GenerateRandomWavePhases();

        ///Generate random wave phases
        void GenerateRandomWavePhases(int seed);

    private:

        ///Generate random wave phases
        void GenerateRandomWavePhases(std::mt19937& seed);

    public:

        std::vector<double> GetWaveFrequencies(FREQUENCY_UNIT unit) const override;

        std::vector<double> GetWaveNumbers() const override { return m_waveNumbers; }

        std::vector<std::vector<double>> GetWaveAmplitudes() const override { return c_amplitude; }

        std::vector<double> GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const override;

        //------------------------------------MAIN GETTERS----------------------------------//

        /// Get the complex wave elevation at the position (x,y,0), of the irregular Airy wave field
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return complex wave elevation, in meters
        std::vector<std::vector<Complex>> GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const override;

        /// Return the complex eulerian fluid particule velocity in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param fc frame convention (NED/NWU)
        /// \return complex eulerian fluid particule velocity, in m/s
        virtual std::vector<mathutils::Vector3d<Complex>> GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const;

        /// Get the wave elevation on the horizontal position (x,y)
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return wave elevation, in meters
        double GetElevation(double x, double y, FRAME_CONVENTION fc) const override;

        /// Return the eulerian fluid particule velocity in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule velocity, in m/s
        Velocity GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const override;

        /// Return the eulerian fluid particule acceleration in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param fc frame convention (NED/NWU)
        /// \return eulerian fluid particule acceleration, in m/s²
        Acceleration GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const override;

        /// Get the pressure at the position (x,y,z) for an irregular Airy wave field.
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \param fc frame convention (NED/NWU)
        /// \return Pressure.
        double GetPressure(double x, double y, double z, FRAME_CONVENTION fc) const final;

        /// Initialize the state of the wave field
        void Initialize() override;

    protected:

        /// Compute the wave directions vector
        void ComputeWaveDirections();

    };

}  // end namespace frydom


#endif //FRYDOM_FRAIRYIRREGULARWAVEFIELD_H
