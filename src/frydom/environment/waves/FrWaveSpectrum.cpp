//
// Created by frongere on 04/10/17.
//

#include "FrWaveSpectrum.h"

namespace frydom {


    std::vector<double>
    FrWaveSpectrum::GetWaveAmplitudes(const unsigned int nb_waves, const double wmin, const double wmax,
                                      FREQ_UNIT unit) const {

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

    void FrJonswapWaveSpectrum::CheckGamma() {
        if (m_gamma < 1. || m_gamma > 10.) {
            // TODO: utiliser un vrai warning sur stderr
            std::cout << "WARNING: Valid values of gamma parameter in Jonswap wave spectrum are between 1 and 10. " << std::endl;
        }
    }

    double FrJonswapWaveSpectrum::Eval(const double w) const {

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

    std::vector<double> FrJonswapWaveSpectrum::Eval(const std::vector<double> w) const {

        std::vector<double> S_w;
        auto nw = w.size();
        S_w.reserve(nw);

        for (double omega : w) {
            S_w.push_back(Eval(omega));
        }

        return S_w;
    }

    void FrCos2sDirectionalWaveSpectrum::CheckSpreadingFactor() {
        // TODO: utiliser un warning ver la stderr
        if (m_spreading_factor < 1. || m_spreading_factor > 100.) {
            std::cout << "The spreading factor of a cos2s directional spectrum model should lie between 1. and 100." << std::endl;
        }
    }

    void
    FrCos2sDirectionalWaveSpectrum::EvalSpreadingFunction(const std::vector<double> &thetaVect, const double theta_mean) {

        c_spreading_fcn.clear();
        c_spreading_fcn.reserve(thetaVect.size());

        double s = m_spreading_factor;
        double dbs = 2. * s;

        double c_s = ( pow(2., dbs - 1.) / M_PI ) * pow(std::tgamma(s + 1.), 2.) / std::tgamma(dbs + 1.);

        for (double theta: thetaVect) {
            c_spreading_fcn.push_back(c_s * pow(cos(0.5 * (theta - theta_mean)), dbs));
        }
    }

    std::vector<double>
    FrCos2sDirectionalWaveSpectrum::Eval(const double w, const std::vector<double> &theta, const double theta_mean,
                                         bool eval_spreading_fcn) {
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

    std::vector<std::vector<double>>
    FrCos2sDirectionalWaveSpectrum::Eval(const std::vector<double> &wVect, const std::vector<double> &theta,
                                         const double theta_mean) {

        EvalSpreadingFunction(theta, theta_mean);

        std::vector<std::vector<double>> S_w_theta;
        S_w_theta.reserve(wVect.size());

        for (auto w: wVect) {
            S_w_theta.push_back(Eval(w, theta, theta_mean, false));
        }

        return S_w_theta;
    }

    std::vector<std::vector<double>>
    FrCos2sDirectionalWaveSpectrum::GetWaveAmplitudes(const unsigned int nb_waves, const double wmin, const double wmax,
                                                      const unsigned int nb_dir, const double theta_min,
                                                      const double theta_max, const double theta_mean) {

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
}  // end namespace frydom