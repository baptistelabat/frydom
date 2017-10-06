//
// Created by frongere on 04/10/17.
//

#ifndef FRYDOM_FRWAVESPECTRUM_H
#define FRYDOM_FRWAVESPECTRUM_H

#include <vector>

#include "frydom/core/FrConstants.h"
#include "frydom/misc/FrLinspace.h"

namespace frydom {

    class FrGenericWaveSpectrum {};

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
                                              FREQ_UNIT unit=RADS) const {

            // Building a linear regular pulsation vector
            auto pulsations = linspace(
                    convert_frequency(wmin, unit, RADS),
                    convert_frequency(wmax, unit, RADS),
                    nb_waves
            );

            double dw = pulsations[1] - pulsations[0];

            auto Sw = Eval(pulsations);

            std::vector<double> wave_ampl;
            for (int i=0; i < nb_waves; ++i) {
                wave_ampl.push_back(sqrt(2. * Sw[i] * dw));
            }

            return wave_ampl;

        }

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

        void CheckGamma() {
            if (m_gamma < 1. || m_gamma > 10.) {
                // TODO: utiliser un vrai warning sur stderr
                std::cout << "WARNING: Valid values of gamma parameter in Jonswap wave spectrum are between 1 and 10. " << std::endl;
            }
        }

        double GetGamma() const { return m_gamma; }

        void SetGamma(const double gamma) {
            m_gamma = gamma;
            CheckGamma();
        }

        double Eval(const double w) const {

            double wp2 = m_peak_pulsation * m_peak_pulsation;
            double wp4 = wp2 * wp2;

            double w4_1 = pow(w, -4);
            double w5_1 = w4_1 / w;

            double hs2 = m_significant_height * m_significant_height;

            double S_w = 0.;
            if (w > 0.) {
                S_w = 0.3125 * hs2 * wp4 * w5_1 * exp(-1.25 * wp4 * w4_1) * (1 - 0.287 * log(m_gamma));

                double a = exp(-pow(w - m_peak_pulsation, 2) / (2. * wp2));
                if (w <= m_peak_pulsation) {
                    a = pow(a, _SIGMA2_1_left);
                } else {
                    a = pow(a, _SIGMA2_1_right);
                }
                S_w *= pow(m_gamma, a);
            }

            return S_w;
        }

        std::vector<double> Eval(const std::vector<double> w) const {

            std::vector<double> S_w;
            auto nw = w.size();
            S_w.reserve(nw);

            for (double omega : w) {
                S_w.push_back(Eval(omega));
            }

            return S_w;
        }

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

        void CheckSpreadingFactor() {
            // TODO: utiliser un warning ver la stderr
            if (m_spreading_factor < 1. || m_spreading_factor > 100.) {
                std::cout << "The spreading factor of a cos2s directional spectrum model should lie between 1. and 100." << std::endl;
            }
        }

        void EvalSpreadingFunction(const std::vector<double> &thetaVect, const double theta_mean) {

            c_spreading_fcn.clear();
            c_spreading_fcn.reserve(thetaVect.size());

            double s = m_spreading_factor;
            double dbs = 2. * s;

            double c_s = ( pow(2., dbs - 1.) / M_PI ) * pow(std::tgamma(s + 1.), 2.) / std::tgamma(dbs + 1.);

            for (double theta: thetaVect) {
                c_spreading_fcn.push_back(c_s * pow(cos(0.5 * (theta - theta_mean)), dbs));
            }
        }

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
                                 const double theta_mean, bool eval_spreading_fcn=true) {
            double S_w = m_wave_spectrum->Eval(w);

            if (eval_spreading_fcn) {
                EvalSpreadingFunction(theta, theta_mean);
            }

            std::vector<double> S_w_theta;
            S_w_theta.reserve(theta.size());
            for (auto d_theta_i: c_spreading_fcn) {
                S_w_theta.push_back(S_w * d_theta_i);
            }
            return S_w_theta;
        }

        std::vector<std::vector<double>> Eval(const std::vector<double>& wVect,
                                              const std::vector<double>& theta,
                                              const double theta_mean) {

            EvalSpreadingFunction(theta, theta_mean);

            std::vector<std::vector<double>> S_w_theta;
            S_w_theta.reserve(wVect.size());

            for (auto w: wVect) {
                S_w_theta.push_back(Eval(w, theta, theta_mean, false));
            }

            return S_w_theta;
        }

//        const unsigned int nb_waves,
//        const double wmin,
//        const double wmax,
//                FREQ_UNIT unit=RADS

        std::vector<std::vector<double>> GetWaveAmplitudes(const unsigned int nb_waves,
                                                           const double wmin,
                                                           const double wmax,
                                                           const unsigned int nb_dir,
                                                           const double theta_min,
                                                           const double theta_max,
                                                           const double theta_mean) {

            // Frequency vector
            auto omega = linspace(wmin, wmax, nb_waves);
            double dw = (wmax-wmin) / nb_waves;
            auto nw = omega.size();

            // Direction vector
            auto theta = linspace(theta_min, theta_max, nb_dir);
            double dtheta = (theta_max-theta_min) / nb_dir;
            auto ntheta = theta.size();

            // Directional spectrum evaluation
            auto S_w_theta = Eval(omega, theta, theta_mean);

            std::vector<std::vector<double>> wave_ampl;
            wave_ampl.reserve(nw);

            std::vector<double> waw;
            for (const auto& S_theta: S_w_theta) {
                waw.clear();
                waw.reserve(ntheta);
                for (double s: S_theta) {
                    waw.push_back(sqrt(2. * s * dw * dtheta));
                }
                wave_ampl.push_back(waw);
            }
            return wave_ampl;

        }

    };



}  // end namespace frydom


#endif //FRYDOM_FRWAVESPECTRUM_H
