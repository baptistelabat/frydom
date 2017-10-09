//
// Created by frongere on 04/10/17.
//

#ifndef FRYDOM_FRWAVESPECTRUM_H
#define FRYDOM_FRWAVESPECTRUM_H

#include <vector>

#include "frydom/core/FrConstants.h"
#include "frydom/misc/FrLinspace.h"

namespace frydom {


    class FrGenericWaveSpectrum {
    public:
        virtual double Eval(const double x) const = 0;
    };


    class FrWaveSpectrum : public FrGenericWaveSpectrum {

    protected:
        double m_significant_height;
        double m_peak_pulsation;

    public:

        FrWaveSpectrum(const double hs, const double tp, const FREQ_UNIT unit=S) :
                m_significant_height(hs),
                m_peak_pulsation(convert_frequency(tp, unit, RADS)) {}

        FrWaveSpectrum(const FrWaveSpectrum& waveSpectrum) :
                m_significant_height(waveSpectrum.m_significant_height),
                m_peak_pulsation(waveSpectrum.m_peak_pulsation) {}  // Copy CTOR


        double GetHs() const { return m_significant_height; }

        double GetTp() const { return convert_frequency(m_peak_pulsation, RADS, S); }

        double GetWp() const { return m_peak_pulsation; }

        double GetFp() const { return convert_frequency(m_peak_pulsation, RADS, HZ); }

        double GetPeakFreq(FREQ_UNIT unit) const {
            return convert_frequency(m_peak_pulsation, RADS, unit);
        }

        void GetFrequencyBandwidth(double& wmin, double& wmax) const {
            // TODO
        }

        virtual double Eval(const double x) const = 0;

        virtual std::vector<double> Eval(const std::vector<double> x) const = 0;  // TODO:utiliser un ChVectorDynamic



        std::vector<double> GetWaveAmplitudes(const unsigned int nb_waves,
                                              const double wmin,
                                              const double wmax,
                                              FREQ_UNIT unit=RADS) const;

//        void ToDirectional() {} // TODO ?

    };


    #define _SIGMA2_1_left (1/(0.07*0.07))
    #define _SIGMA2_1_right (1/(0.09*0.09))

    class FrJonswapWaveSpectrum : public FrWaveSpectrum {

    private:
        double m_gamma;

    public:

        FrJonswapWaveSpectrum(const double hs, const double tp, const FREQ_UNIT unit=S, const double gamma=3.3) :
                FrWaveSpectrum(hs, tp, unit),
                m_gamma(gamma) {
            CheckGamma();
        }

        FrJonswapWaveSpectrum(const FrJonswapWaveSpectrum& waveSpectrum) :
                m_gamma(waveSpectrum.m_gamma),
                FrWaveSpectrum(waveSpectrum)
                {}  // Copy CTOR

        void CheckGamma();

        double GetGamma() const { return m_gamma; }

        void SetGamma(const double gamma) {
            m_gamma = gamma;
            CheckGamma();
        }

        double Eval(const double w) const;

        std::vector<double> Eval(const std::vector<double> w) const;

    };


    class FrDirectionalWaveSpectrum : public FrGenericWaveSpectrum {

    protected:
        std::unique_ptr<FrWaveSpectrum> m_wave_spectrum;

    public:
        explicit FrDirectionalWaveSpectrum(FrWaveSpectrum* waveSpectrum) : m_wave_spectrum(waveSpectrum) {}

        double GetHs() const { return m_wave_spectrum->GetHs(); }

        double GetTp() const { return m_wave_spectrum->GetTp(); }

        double GetWp() const { return m_wave_spectrum->GetWp(); }

        double GetFp() const { return m_wave_spectrum->GetFp(); }

        double GetPeakFreq(FREQ_UNIT unit) const {
            return m_wave_spectrum->GetPeakFreq(unit);
        }

        virtual std::vector<double> Eval(const double w,
                                         const std::vector<double>& theta,
                                         const double theta_mean,
                                         bool eval_spreading_fcn=true) = 0;

        virtual std::vector<std::vector<double>> Eval(const std::vector<double>& wVect,
                                                      const std::vector<double>& theta,
                                                      const double theta_mean) = 0;

        virtual std::vector<std::vector<double>> GetWaveAmplitudes(const unsigned int nb_waves,
                                                                   const double wmin,
                                                                   const double wmax,
                                                                   const unsigned int nb_dir,
                                                                   const double theta_min,
                                                                   const double theta_max,
                                                                   const double theta_mean) = 0;
    };


    class FrCos2sDirectionalWaveSpectrum : public FrDirectionalWaveSpectrum {

    private:
        double m_spreading_factor = 10.;

        std::vector<double> c_spreading_fcn;  // Cached

        void CheckSpreadingFactor();

        void EvalSpreadingFunction(const std::vector<double> &thetaVect, const double theta_mean);

    public:

        explicit FrCos2sDirectionalWaveSpectrum(FrWaveSpectrum* waveSpectrum, const double spreading_factor=10.) :
                m_spreading_factor(spreading_factor),
                FrDirectionalWaveSpectrum(waveSpectrum) {
            CheckSpreadingFactor();
        }

        double GetSpreadingFactor() const { return m_spreading_factor; }

        void SetSpreadingFactor(const double spreading_factor) {
            m_spreading_factor = spreading_factor;
            CheckSpreadingFactor();
        }

        std::vector<double> Eval(const double w,
                                 const std::vector<double>& theta,
                                 const double theta_mean, bool eval_spreading_fcn=true);

        std::vector<std::vector<double>> Eval(const std::vector<double>& wVect,
                                              const std::vector<double>& theta,
                                              const double theta_mean);

        std::vector<std::vector<double>> GetWaveAmplitudes(const unsigned int nb_waves,
                                                           const double wmin,
                                                           const double wmax,
                                                           const unsigned int nb_dir,
                                                           const double theta_min,
                                                           const double theta_max,
                                                           const double theta_mean);

    };


}  // end namespace frydom


#endif //FRYDOM_FRWAVESPECTRUM_H
