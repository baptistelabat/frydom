//
// Created by Lucas Letournel on 03/12/18.
//

#ifndef FRYDOM_FRAIRYIRREGULARWAVEFIELD_H
#define FRYDOM_FRAIRYIRREGULARWAVEFIELD_H
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"


namespace frydom {

    //Forward Declaration
    class FrFreeSurface_;


    class FrAiryIrregularWaveField : public FrWaveField_ {
    private:

        double m_minFreq = 0.05;
        double m_maxFreq = 2.;
        unsigned int m_nbFreq = 40;

//        double m_minDir;
//        double m_maxDir;
        unsigned int m_nbDir = 1;

        double m_meanDir = 0;

        /// Wave spectrum, by default JONSWAP (Hs=3m,Tp=9s,Gamma=3.3)
        std::unique_ptr<FrWaveSpectrum> m_waveSpectrum;

        std::vector<double> m_waveDirections;
        std::vector<double> m_waveFrequencies;
        std::vector<double> m_waveNumbers;
//        std::vector<std::complex<double>> m_emjwt;

        /// Table of wave phases,of dimensions (m_nbDir,m_nbFreq)
        /// made unique to check at initialize() if wavePhases were given by the users,
        /// or if they need to be randomly generated.
        std::unique_ptr<std::vector<std::vector<double>>> m_wavePhases;

        ///< Vertical scale velocity factor with stretching
        std::unique_ptr<FrKinematicStretching_> m_verticalFactor;
        
    public:

        /// Default constructor
        /// \param freeSurface pointer to the free surface, to which the wave field belongs
        explicit FrAiryIrregularWaveField(FrFreeSurface_* freeSurface);

        void SetWaveFrequencies(double minFreq, double maxFreq, unsigned int nbFreq);

        void SetMeanWaveDirectionAngle(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);
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


        std::vector<std::vector<double>>* GetWavePhases() const {return m_wavePhases.get();}

        void SetWavePhases(std::vector<std::vector<double>>& wavePhases);

        void SetDirectionalParameters(unsigned int nbDir, double spreadingFactor);

        void SetStretching(FrStretchingType type);

//        void SetWaveSpectrum(WAVE_SPECTRUM_TYPE type);

        FrJonswapWaveSpectrum* SetJonswapWaveSpectrum(double Hs, double Tp, FREQUENCY_UNIT unit=S, double gamma=3.3);

        FrPiersonMoskowitzWaveSpectrum* SetPiersonMoskovitzWaveSpectrum(double Hs, double Tp, FREQUENCY_UNIT unit=S);

        FrTestWaveSpectrum* SetTestWaveSpectrum();

        FrWaveSpectrum* GetWaveSpectrum() const;

        void GenerateRandomWavePhases();

        std::vector<std::vector<Complex>> GetComplexElevation(double x, double y) const;

        std::vector<mathutils::Vector3d<Complex>> GetComplexVelocity(double x, double y, double z) const;

        double GetElevation(double x, double y) const override;

        /// Return the eulerian fluid particule velocity (in global frame)
        Velocity GetVelocity(double x, double y, double z) const override;

        /// Return the eulerian fluid particule acceleration (in global frame)
        Acceleration GetAcceleration(double x, double y, double z) const override;

        void Initialize() override;

        void Update(double time) final;;

    protected:
        void ComputeWaveDirections();

    };

};


#endif //FRYDOM_FRAIRYIRREGULARWAVEFIELD_H
