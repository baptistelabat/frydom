// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRWAVEFIELD_H
#define FRYDOM_FRWAVEFIELD_H


//#include <vector>
//#include <complex>
//#include <random>

#include "frydom/core/common/FrObject.h"
#include "frydom/core/math/FrVector.h"

#include "chrono/core/ChVector.h"

#include "FrWaveSpectrum.h"
#include "FrKinematicStretching.h"

#include "frydom/core/math/FrComplex.h"


//#include "frydom/core/FrConstants.h"
//#include "FrWaveSpectrum.h"
//#include "FrWaveDispersionRelation.h"
//#include "frydom/utils/FrUtils.h"
//
//#include "frydom/environment/waves/FrKinematicStretching.h"


namespace frydom {

    // Forward declarations
    class FrWaveProbe;
    class FrLinearWaveProbe;
    class FrFlowSensor;
    class FrLinearFlowSensor;


    /**
     * \class FrRamp
     * \brief Class for dealing with the ramp function.
     */
    class FrRamp : public FrObject {  // TODO: placer cette classe dans son propre fichier

    private:
        bool m_active = true;

        bool m_increasing = true;
        double m_t0 = 0.;
        double m_t1 = 20.;

        double m_min = 0.;
        double m_max = 1.;

        double c_a = 0.;
        double c_b = 1.;


    public:

        void SetDuration(double duration);

        void SetIncrease();

        void SetDecrease();

        void SetMinVal(double minVal);

        void SetMaxVal(double maxVal);

        bool IsActive();

        void Deactivate();

        void Initialize();

        void Apply(const double t, double& value);

        void Apply(const double t, chrono::ChVector<double>& vect);

        virtual void StepFinalize() override;

    };



    // FrWaveField declarations


    enum WAVE_MODEL { // TODO: passer l'enum dans la classe
        NO_WAVES,
        LINEAR_WAVES
    };


    // Forward declarations
    class FrFreeSurface;

    /**
     * \class FrWaveField
     * \brief Class for defining a wave field (null or linear).
     */
    class FrWaveField : public FrObject {

    protected:

        FrFreeSurface* m_freeSurface;

        // FIXME static const WAVE_MODEL m_waveModel;
        WAVE_MODEL m_waveModel = NO_WAVES;

        double m_time = 0.;

        std::unique_ptr<FrWaveSpectrum> m_waveSpectrum = nullptr;

        std::shared_ptr<FrRamp> m_waveRamp;  // TODO : passer en unique

        double m_depth = 0.;                     ///< Water depth (m) // TODO: aller chercher dans le seabed...
        bool m_infinite_depth = true;              ///< if true water depth is considered as infinite

    public:

        explicit  FrWaveField(FrFreeSurface* freeSurface);

        ~FrWaveField() = default;

        virtual void Update(double time);

        double GetTime() const;

        void SetWaveSpectrum(WAVE_SPECTRUM_TYPE type);

        WAVE_MODEL GetWaveModel() const;

        std::shared_ptr<FrRamp> GetWaveRamp() const;

        virtual double GetElevation(double x, double y) const = 0;

        /// Return the eulerian fluid particule velocity in global reference frame (implemented in child)
        virtual chrono::ChVector<double> GetVelocity(double x, double y, double z) const = 0;

        /// Return the eulerian flow velocity. Return null vector if the point is upper the free surface elevation
        virtual chrono::ChVector<double> GetVelocity(double x, double y, double z, bool cutoff) const;


        /// Return the eulerian fluid particule velocity in global reference frame (from vector position)
        virtual chrono::ChVector<double> GetVelocity(chrono::ChVector<double> vect) const;

        /// Return the eulerian fluid particule acceleration in global reference frame (implemented in child)
        virtual chrono::ChVector<double> GetAcceleration(double x, double y, double z) const = 0;

        virtual chrono::ChVector<double> GetAcceleration(double x, double y, double z, bool cutoff) const;

        /// Return the eulerian fluid particule acceleration in global reference frame (from vector position)
        virtual chrono::ChVector<double> GetAcceleration(chrono::ChVector<double> vect) const;

        virtual std::vector<std::vector<double>> GetElevation(const std::vector<double>& xVect,
                                                              const std::vector<double>& yVect) const = 0;


        virtual std::vector<std::vector<std::vector<chrono::ChVector<double>>>> GetVelocityGrid(const std::vector<double>& xvect,
                                                                  const std::vector<double>& yvect,
                                                                  const std::vector<double>& zvect) const = 0;

//        virtual FrFlowSensor* SetFlowSensor(double x, double y, double z) const;
//        virtual FrFlowSensor* SetFlowSensor(chrono::ChVector<> pos) const;

        void Initialize() override;

        void StepFinalize() override;

    };


    /**
     * \class FrNullWaveField
     * \brief Class for defining a null wave field.
     */
    class FrNullWaveField : public FrWaveField {

    private:
        //FIXME : static const WAVE_MODEL m_waveModel = NO_WAVES;
        WAVE_MODEL m_waveModel = NO_WAVES;

    public:
        explicit FrNullWaveField(FrFreeSurface* freeSurface);

        double GetElevation(double x, double y) const final;

        chrono::ChVector<double> GetVelocity(double x, double y, double z) const final;

        chrono::ChVector<double> GetAcceleration(double x, double y, double z) const final;

        std::vector<std::vector<double>> GetElevation(const std::vector<double>& xVect,
                                                      const std::vector<double>& yVect) const final;

        std::vector<std::vector<std::vector<chrono::ChVector<double>>>>
        GetVelocityGrid(const std::vector<double>& xvect,
                    const std::vector<double>& yvect,
                    const std::vector<double>& zvect) const final;


        std::shared_ptr<FrWaveProbe> NewWaveProbe(double x, double y);

    };

    enum LINEAR_WAVE_TYPE {
        LINEAR_REGULAR,
        LINEAR_IRREGULAR,
        LINEAR_DIRECTIONAL
    };

    // =================================================================================================================

    /**
     * \class FrLinearWaveField
     * \brief Class for defining a linear wave field.
     */
    class FrLinearWaveField : public FrWaveField {

    private:
        //FIXME : static const WAVE_MODEL m_waveModel = LINEAR_WAVES;
        WAVE_MODEL m_waveModel = LINEAR_WAVES;

        LINEAR_WAVE_TYPE m_linearWaveType = LINEAR_DIRECTIONAL;

        double m_minFreq = 0.;
        double m_maxFreq = 2.;
        unsigned int m_nbFreq = 40;

        double m_minDir = -180. * MU_PI_180;
        double m_maxDir = 165. * MU_PI_180;
        unsigned int m_nbDir = 20;

        double m_meanDir = 0.;

        // For regular wave field
        double m_height = 0.;
        double m_period = 0.;

        std::vector<double> c_waveFrequencies;
        std::vector<double> c_waveNumbers;
        std::vector<std::complex<double>> c_emjwt;

        std::unique_ptr<std::vector<std::vector<double>>> m_wavePhases; // Not used in regular wave field

        std::vector<std::shared_ptr<FrLinearWaveProbe>> m_waveProbes;
        std::vector<std::shared_ptr<FrLinearFlowSensor>> m_flowSensor;

        std::shared_ptr<FrKinematicStretching> m_verticalFactor;        ///< Vertical scale velocity factor with stretching


    public:

        explicit FrLinearWaveField(FrFreeSurface* freeSurface, LINEAR_WAVE_TYPE type);

        LINEAR_WAVE_TYPE GetType() const;

        void SetType(LINEAR_WAVE_TYPE type);

        void SetRegularWaveHeight(double height);

        void SetRegularWavePeriod(double period, FREQUENCY_UNIT unit=S);

        void SetStretching(FrStretchingType type);

        unsigned int GetNbFrequencies() const;

        double GetMinFrequency() const;

        double GetMaxFrequency() const;

        unsigned int GetNbWaveDirections() const;

        double GetMinWaveDirection() const;

        double GetMaxWaveDirection() const;

        double GetMeanWaveDirection(ANGLE_UNIT unit=DEG) const;

        void SetMeanWaveDirection(const double meanDirection, ANGLE_UNIT unit=DEG);

        std::vector<double> GetWaveDirections(ANGLE_UNIT unit=DEG) const;

        void SetWaveDirections(const double minDir, const double maxDir, const unsigned int nbDir, ANGLE_UNIT unit=DEG);

        std::vector<double> GetWaveFrequencies(FREQUENCY_UNIT unit = RADS) const;

        void SetWavePulsations(const double minFreq, const double maxFreq, const unsigned int nbFreq,
                               FREQUENCY_UNIT unit=RADS);

        void Initialize() override;

        std::vector<std::vector<double>>* GetWavePhases() const;

        void SetWavePhases(std::vector<std::vector<double>>& wavePhases);

        void GenerateRandomWavePhases();

        std::vector<double> GetWaveLengths() const;

        std::vector<double> GetWaveNumbers() const;

        std::vector<std::vector<double>> _GetWaveAmplitudes() const;

        std::vector<std::vector<std::complex<double>>>
        GetCmplxElevation(const double x, const double y, bool steady=false) const;


        std::vector<chrono::ChVector<std::complex<double>>>
        GetSteadyVelocity(const double x, const double y, const double z) const;

        std::vector<std::complex<double>> GetSteadyElevation(const double x, const double y) const;

        FrWaveSpectrum* GetWaveSpectrum() const;

        void SetReturnPeriod();

        double GetReturnPeriod() const;

        std::shared_ptr<FrLinearWaveProbe> NewWaveProbe(double x, double y);

        /// Add and return a new wave probe to the wave field with default position (global origine)
        std::shared_ptr<FrLinearWaveProbe> NewWaveProbe();

//        /// Add an already existing wave probe to the wev field
//        void AddWaveProbe(std::shared_ptr<FrLinearWaveProbe> waveProbe);

        /// Create a new flow sensor linked to the wave field
        std::shared_ptr<FrLinearFlowSensor> NewFlowSensor(double x, double y, double z);

        void Update(double time) override;

        // TODO: renvoyer un pointeur partagé ??
        const std::vector<std::complex<double>>& GetTimeCoeffs() const;

        const std::vector<std::complex<double>>& GetTimeCoeffs(chrono::ChVector<double> vel) const;

        /// Return the time derivative of the temporal factor
        std::vector<std::complex<double>> GetTimeCoeffsDt() const;

        double GetElevation(double x, double y) const;

        /// Return the eulerian fluid particule velocity (in global frame)
        chrono::ChVector<double> GetVelocity(double x, double y, double z) const override;

        /// Return the eulerian fluid particule acceleration (in global frame)
        chrono::ChVector<double> GetAcceleration(double x, double y, double z) const override;


        std::vector<std::vector<double>> GetElevation(const std::vector<double>& xVect,
                                                      const std::vector<double>& yVect) const;


        /// Return the flow velocity vector field in a grid [xvect x yvect x zvect]
        std::vector<std::vector<std::vector<chrono::ChVector<double>>>> GetVelocityGrid(const std::vector<double>& xvect,
                                                          const std::vector<double>& yvect,
                                                          const std::vector<double>& zvect) const override;

    private:

        double Fz(const double& z, const double& k) const;

        inline double dFz(const double& z, const double& k) const;

    };


//    std::shared_ptr<FrWaveField> MakeWaveField(FrWaveField::LINEAR_WAVE_TYPE waveType) {
//
//        if (waveType == FrWaveField::LINEAR_REGULAR
//            || waveType == FrWaveField::LINEAR_IRREGULAR
//            || waveType == FrWaveField::LINEAR_DIRECTIONAL) {
//
//            return std::make_shared<FrLinearWaveField>(waveType);
//
//        }
//
//    }















    // REFACTORING ------------->>>>>>>>>>>>>>>>>>




    // FrWaveField declarations

    // Forward declarations
    class FrFreeSurface_;
    class FrRamp_;

    /**
     * \class FrWaveField_
     * \brief Class for defining a wave field (null or linear).
     */
    class FrWaveField_ : public FrObject {

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

////    enum LINEAR_WAVE_TYPE {
////        LINEAR_REGULAR,
////        LINEAR_IRREGULAR,
////        LINEAR_DIRECTIONAL
////    };
//
//    // =================================================================================================================
//
//    // Forward declarations
//    class FrLinearWaveProbe_;
//    class FrLinearFlowSensor_;
//
//
//    class FrLinearWaveField_ : public FrWaveField_ {
//
//    private:
//        //FIXME : static const WAVE_MODEL m_waveModel = LINEAR_WAVES;
//        WAVE_MODEL m_waveModel = LINEAR_WAVES;
//
//        LINEAR_WAVE_TYPE m_linearWaveType = LINEAR_DIRECTIONAL;
//
//        double m_minFreq = 0.;
//        double m_maxFreq = 2.;
//        unsigned int m_nbFreq = 40;
//
//        double m_minDir = -180. * MU_PI_180;
//        double m_maxDir = 165. * MU_PI_180;
//        unsigned int m_nbDir = 20;
//
//        double m_meanDir = 0.;
//
//        // For regular wave field
//        double m_height = 0.;
//        double m_period = 0.;
//
//        std::unique_ptr<FrWaveSpectrum> m_waveSpectrum = nullptr;
//
//        std::vector<double> c_waveFrequencies;
//        std::vector<double> c_waveNumbers;
//        std::vector<std::complex<double>> c_emjwt;
//
//        std::unique_ptr<std::vector<std::vector<double>>> m_wavePhases; // Not used in regular wave field
//
//        std::vector<std::shared_ptr<FrLinearWaveProbe_>> m_waveProbes;
//        std::vector<std::shared_ptr<FrLinearFlowSensor_>> m_flowSensor;
//
//        std::unique_ptr<FrKinematicStretching_> m_verticalFactor;        ///< Vertical scale velocity factor with stretching
//
//
//    public:
//
//        explicit FrLinearWaveField_(FrFreeSurface_* freeSurface, LINEAR_WAVE_TYPE type);
//
//        LINEAR_WAVE_TYPE GetType() const;
//
//        void SetType(LINEAR_WAVE_TYPE type);
//
////        void SetWaveHeight(double height);
////
////        void SetWavePeriod(double period, FREQUENCY_UNIT unit=S);
//
//        void SetStretching(FrStretchingType type);
//
//        unsigned int GetNbFrequencies() const;
//
//        double GetMinFrequency() const;
//
//        double GetMaxFrequency() const;
//
//        unsigned int GetNbWaveDirections() const;
//
//        double GetMinWaveDirection() const;
//
//        double GetMaxWaveDirection() const;
//
//        double GetMeanWaveDirection(ANGLE_UNIT unit=DEG) const;
//
//        void SetMeanWaveDirection(double meanDirection, ANGLE_UNIT unit=DEG);
//
//        std::vector<double> GetWaveDirections(ANGLE_UNIT unit=DEG) const;
//
//        void SetWaveDirections(double minDir, double maxDir, unsigned int nbDir, ANGLE_UNIT unit=DEG);
//
//        std::vector<double> GetWaveFrequencies(FREQUENCY_UNIT unit = RADS) const;
//
//        void SetWavePulsations(double minFreq, double maxFreq, unsigned int nbFreq,
//                               FREQUENCY_UNIT unit=RADS);
//
//        void Initialize() override;
//
//        std::vector<std::vector<double>>* GetWavePhases() const;
//
//        void SetWavePhases(std::vector<std::vector<double>>& wavePhases);
//
//        void GenerateRandomWavePhases();
//
//        std::vector<double> GetWaveLengths() const;
//
//        std::vector<double> GetWaveNumbers() const;
//
//        std::vector<std::vector<double>> _GetWaveAmplitudes() const;
//
//        std::vector<std::vector<std::complex<double>>>
//        GetCmplxElevation(double x, double y, bool steady=false) const;
//
//
//        std::vector<chrono::ChVector<std::complex<double>>>
//        GetSteadyVelocity(double x, double y, double z) const;
//
//        std::vector<std::complex<double>> GetSteadyElevation(double x, double y) const;
//
//
//        void SetWaveSpectrum(WAVE_SPECTRUM_TYPE type);
//
//        FrWaveSpectrum* GetWaveSpectrum() const;
//
////        void SetReturnPeriod();
////
////        double GetReturnPeriod() const;
//
//        std::shared_ptr<FrLinearWaveProbe_> NewWaveProbe(double x, double y);
//
//        /// Add and return a new wave probe to the wave field with default position (global origine)
//        std::shared_ptr<FrLinearWaveProbe_> NewWaveProbe();
//
////        /// Add an already existing wave probe to the wev field
////        void AddWaveProbe(std::shared_ptr<FrLinearWaveProbe> waveProbe);
//
//        /// Create a new flow sensor linked to the wave field
//        std::shared_ptr<FrLinearFlowSensor_> NewFlowSensor(FrLinearWaveField_* waveField, double x, double y, double z);
//
//        void Update(double time) override;
//
//        // TODO: renvoyer un pointeur partagé ??
//        const std::vector<std::complex<double>>& GetTimeCoeffs() const;
//
//        const std::vector<std::complex<double>>& GetTimeCoeffs(chrono::ChVector<double> vel) const;
//
//        /// Return the time derivative of the temporal factor
//        std::vector<std::complex<double>> GetTimeCoeffsDt() const;
//
//        double GetElevation(double x, double y) const override;
//
//        /// Return the eulerian fluid particule velocity (in global frame)
//        Velocity GetVelocity(double x, double y, double z) const override;
//
//        /// Return the eulerian fluid particule acceleration (in global frame)
//        Acceleration GetAcceleration(double x, double y, double z) const override;
//
//    private:
//
//        double Fz(double z, double k) const;
//
//        double dFz(double z, double k) const;
//
//    };
//
//
////    std::shared_ptr<FrWaveField> MakeWaveField(FrWaveField::LINEAR_WAVE_TYPE waveType) {
////
////        if (waveType == FrWaveField::LINEAR_REGULAR
////            || waveType == FrWaveField::LINEAR_IRREGULAR
////            || waveType == FrWaveField::LINEAR_DIRECTIONAL) {
////
////            return std::make_shared<FrLinearWaveField>(waveType);
////
////        }
////
////    }


}  // end namespace frydom

#endif //FRYDOM_FRWAVEFIELD_H
