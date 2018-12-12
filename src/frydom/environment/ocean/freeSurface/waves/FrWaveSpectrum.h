//
// Created by frongere on 04/10/17.
//

#ifndef FRYDOM_FRWAVESPECTRUM_H
#define FRYDOM_FRWAVESPECTRUM_H

#include <vector>
#include "MathUtils/MathUtils.h"

#include "frydom/core/FrObject.h"

using namespace mathutils;

namespace frydom {

    // =================================================================================================================

    enum WaveDirectionalModelType {
        NONE,
        COS2S,
        DIRTEST
    };

    // =================================================================================================================

    class FrWaveDirectionalModel {
    public:

        virtual WaveDirectionalModelType GetType() const  = 0;

        void UpdateSpreadingFunction(const unsigned int nb_dir,
                                             const double theta_min,
                                             const double theta_max,
                                             const double theta_mean) {
            UpdateSpreadingFunction(linspace(theta_min, theta_max, nb_dir), theta_mean);
        };

        virtual void UpdateSpreadingFunction(const std::vector<double>& thetaVect,
                                             const double theta_mean) = 0;


        virtual std::vector<double> GetSpreadingFunction() const = 0;

    };

    // =================================================================================================================

    class FrCos2sDirectionalModel : public FrWaveDirectionalModel {

    private:

        double m_spreading_factor = 10.;

        // Cached
        std::vector<double> c_spreading_fcn;
        double c_DTheta = 0.;

        void CheckSpreadingFactor();

    public:

        explicit FrCos2sDirectionalModel(const double spreading_factor=10.);

        WaveDirectionalModelType GetType() const override;

        double GetSpreadingFactor() const;

        void SetSpreadingFactor(const double spreading_factor);

        void UpdateSpreadingFunction(const std::vector<double>& thetaVect, const double theta_mean) override;

        std::vector<double> GetSpreadingFunction() const override;

        double GetDTheta() const;

    };

    // =================================================================================================================
    // For test use only
    class FrTestDirectionalModel : public FrWaveDirectionalModel {
    private:
        std::vector<double> c_spreading_fcn;
    public:

        WaveDirectionalModelType GetType() const override;

        void UpdateSpreadingFunction(const std::vector<double>& thetaVect,
                                     double theta_mean) override;

        std::vector<double> GetSpreadingFunction() const override;
    };


    // =================================================================================================================



    enum WAVE_SPECTRUM_TYPE {
        JONSWAP,
        PIERSON_MOSKOWITZ
    };

    class FrWaveSpectrum : public FrObject {

    protected:
        double m_significant_height = 3.;
        double m_peak_pulsation = S2RADS(9.);

        WaveDirectionalModelType m_dir_model_type = NONE;
        std::unique_ptr<FrWaveDirectionalModel> m_directional_model = nullptr;


    public:

        FrWaveSpectrum() = default;

        /// Constructor
        FrWaveSpectrum(const double hs, const double tp, const FREQUENCY_UNIT unit=S);;

        void SetCos2sDirectionalModel(double spreadingFactor);

        /// Set the directional model to use from type
        void SetDirectionalModel(WaveDirectionalModelType model);

        /// Set the directional model to use from object
        void SetDirectionalModel(FrWaveDirectionalModel* dir_model);

        FrWaveDirectionalModel* GetDirectionalModel() const;

        void DirectionalON(WaveDirectionalModelType model=COS2S);

        void DirectionalOFF();

        double GetHs() const;

        void SetHs(double Hs);

        double GetTp() const;

        void SetTp(double Tp);

        double GetWp() const;

        void SetWp(double Wp);

        double GetFp() const;

        void SetFp(double Fp);

        double GetPeakFreq(FREQUENCY_UNIT unit) const;

        /// Eval the spectrum at one frequency
        /// Must be implemented into each wave spectrum
        virtual double Eval(double w) const = 0;

        /// Eval the spectrum at a vector of frequencies
        virtual std::vector<double> Eval(const std::vector<double> wVect) const;

        std::vector<double> vectorDiscretization(std::vector<double> vect);

        /// Get the wave amplitudes for a given regular frequency discretization
        virtual std::vector<double> GetWaveAmplitudes(std::vector<double> waveFrequencies);


        /// Get the wave amplitudes for a given regular frequency discretization
        virtual std::vector<std::vector<double>> GetWaveAmplitudes(std::vector<double> waveFrequencies, std::vector<double> waveDirections);


        /// Get the wave amplitudes for a given regular frequency discretization
        virtual std::vector<double> GetWaveAmplitudes(const unsigned int nb_waves,
                                                      const double wmin,
                                                      const double wmax);

        virtual std::vector<std::vector<double>> GetWaveAmplitudes(const unsigned int nb_waves,
                                                                   const double wmin,
                                                                   const double wmax,
                                                                   const unsigned int nb_dir,
                                                                   const double theta_min,
                                                                   const double theta_max,
                                                                   const double theta_mean);

        void Initialize() override {}

        void StepFinalize() override {}

    };

    // =================================================================================================================
    // FIXME: est-ce vraiment critique que d'avoir les 2 choses suivantes en macro ???
    #define _SIGMA2_1_left (1./(0.07*0.07))
    #define _SIGMA2_1_right (1./(0.09*0.09))

    class FrJonswapWaveSpectrum : public FrWaveSpectrum {

    private:
        double m_gamma = 3.3;

    public:

        FrJonswapWaveSpectrum() = default;

        FrJonswapWaveSpectrum(const double hs, const double tp, const FREQUENCY_UNIT unit=S, const double gamma=3.3);

        void CheckGamma();

        double GetGamma() const;

        void SetGamma(const double gamma);

        double Eval(const double w) const final;

    };



    // =================================================================================================================

    class FrPiersonMoskowitzWaveSpectrum : public FrWaveSpectrum {

    public:

        FrPiersonMoskowitzWaveSpectrum() = default;

        FrPiersonMoskowitzWaveSpectrum(const double hs, const double tp, const FREQUENCY_UNIT unit=S);

        double Eval(const double w) const final;

    };


    // =================================================================================================================
    // For test use only
    class FrTestWaveSpectrum : public FrWaveSpectrum {
    public:
        FrTestWaveSpectrum() = default;
        double Eval(const double w) const final;
        
    };
    
    
    std::unique_ptr<FrWaveSpectrum> MakeWaveSpectrum(WAVE_SPECTRUM_TYPE type);


}  // end namespace frydom


#endif //FRYDOM_FRWAVESPECTRUM_H
