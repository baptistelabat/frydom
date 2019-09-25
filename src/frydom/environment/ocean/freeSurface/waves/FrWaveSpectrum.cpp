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


#include <frydom/core/common/FrException.h>
#include "FrWaveSpectrum.h"

#include "MathUtils/VectorGeneration.h"

namespace frydom {

    // FrWaveDirectionalModel descriptions

    std::vector<double>
    FrWaveDirectionalModel::GetSpreadingFunction(const std::vector<double> &thetaVect, double theta_mean) {
        std::vector<double> spreading_fcn;
        spreading_fcn.clear();
        spreading_fcn.reserve(thetaVect.size());
        double spreadEval;

        for (double theta: thetaVect) {
            spreadEval = Eval(theta,theta_mean);
            spreading_fcn.push_back(spreadEval);
        }
        return spreading_fcn;
    }

    void FrWaveDirectionalModel::GetDirectionBandwidth(double &theta_min, double &theta_max, double theta_mean) const {
        double threshold = 0.01;

        theta_min = dichotomySearch(theta_mean,threshold);
        double Dtheta = theta_mean - theta_min;
        theta_max = theta_min + 2.*Dtheta;
    }

    double FrWaveDirectionalModel::dichotomySearch(double theta_mean, double threshold) const {
        double theta_min = theta_mean - M_PI;
        double theta_max = theta_mean;

        double theta_result, epsilon = 1.E-10;

        while(fabs(theta_min-theta_max)>epsilon*epsilon)
        {
            theta_result=(theta_min+theta_max)/2.0f;
            if (fabs(Eval(theta_result,theta_mean) - threshold) < epsilon)
            {
                return theta_result;
            }
            else
            {
                if ((Eval(theta_max,theta_mean) - threshold) * (Eval(theta_result,theta_mean) - threshold) < 0.0f)
                    theta_min=theta_result;
                else
                    theta_max=theta_result;
            }
        }
        throw FrException("FrWaveDirectionalModel::Bisection Search  no convergence");
    }

    // FrCos2sDirectionalModel descriptions

    void FrCos2sDirectionalModel::CheckSpreadingFactor() {
        // TODO: utiliser un warning ver la stderr
        if (m_spreading_factor < 1. || m_spreading_factor > 100.) {
            std::cout << "The spreading factor of a cos2s directional spectrum model should lie between 1. and 100." << std::endl;
        }
    }

    FrCos2sDirectionalModel::FrCos2sDirectionalModel(double spreading_factor) : m_spreading_factor(spreading_factor) {
        CheckSpreadingFactor();
        EvalCs();
    }

    WAVE_DIRECTIONAL_MODEL FrCos2sDirectionalModel::GetType() const { return COS2S; }

    double FrCos2sDirectionalModel::GetSpreadingFactor() const { return m_spreading_factor; }

    double FrCos2sDirectionalModel::Eval(double theta, double theta_mean) const {
        return c_s * pow(cos(0.5 * (theta - theta_mean)), 2. * m_spreading_factor);
    }

    void FrCos2sDirectionalModel::SetSpreadingFactor(const double spreading_factor) {
        m_spreading_factor = spreading_factor;
        CheckSpreadingFactor();
        EvalCs();
    }

    void FrCos2sDirectionalModel::EvalCs() {
        double s = m_spreading_factor;
        double two_s = 2. * s;
        c_s = ( pow(2., two_s - 1.) / M_PI ) * pow(std::tgamma(s + 1.), 2.) / std::tgamma(two_s + 1.);
    }

    // =================================================================================================================
    // FrTestDirectionalModel descriptions

    WAVE_DIRECTIONAL_MODEL FrTestDirectionalModel::GetType() const { return DIRTEST; }

    double FrTestDirectionalModel::Eval(double theta, double theta_mean) const {
        return 1.;
    }


    // =================================================================================================================
    // FrWaveSpectrum descriptions

    FrWaveSpectrum::FrWaveSpectrum(double hs, double tp) :
            m_significant_height(hs),
            m_peak_frequency(convert_frequency(tp, mathutils::S, mathutils::RADS)) {}

    void FrWaveSpectrum::SetCos2sDirectionalModel(double spreadingFactor) {
        m_dir_model_type = COS2S;
        m_directional_model = std::make_unique<FrCos2sDirectionalModel>(spreadingFactor);
    }

    void FrWaveSpectrum::SetDirectionalModel(WAVE_DIRECTIONAL_MODEL model) {
        switch (model) {
            case NONE:
                DirectionalOFF();
                break;
            case COS2S:
                m_dir_model_type = COS2S;
                m_directional_model = std::make_unique<FrCos2sDirectionalModel>();
                break;
            case DIRTEST:
                m_dir_model_type = DIRTEST;
                m_directional_model = std::make_unique<FrTestDirectionalModel>();
        }
    }

    void FrWaveSpectrum::SetDirectionalModel(FrWaveDirectionalModel *dir_model) {
        m_dir_model_type = dir_model->GetType();
        m_directional_model = std::unique_ptr<FrWaveDirectionalModel>(dir_model);
    }

    FrWaveDirectionalModel *FrWaveSpectrum::GetDirectionalModel() const {
        return m_directional_model.get();
    }

    void FrWaveSpectrum::DirectionalON(WAVE_DIRECTIONAL_MODEL model) {
        SetDirectionalModel(model);
    }

    void FrWaveSpectrum::DirectionalOFF() {
        m_dir_model_type = NONE;
        m_directional_model = nullptr;
    }

    double FrWaveSpectrum::GetHs() const { return m_significant_height; }

    void FrWaveSpectrum::SetHs(double Hs) { m_significant_height = Hs; }

    double FrWaveSpectrum::GetPeakFreq(FREQUENCY_UNIT unit) const {
        return convert_frequency(m_peak_frequency, mathutils::RADS, unit);
    }

    void FrWaveSpectrum::SetPeakFreq(double Fp, FREQUENCY_UNIT unit) {
        m_peak_frequency = convert_frequency(Fp, unit, mathutils::RADS);
    }

    void FrWaveSpectrum::SetHsTp(double Hs, double Tp, FREQUENCY_UNIT unit) {
        SetHs(Hs);
        SetPeakFreq(Tp,unit);
    }

    std::vector<double> FrWaveSpectrum::Eval(const std::vector<double> &wVect) const {
        std::vector<double> S_w;
        auto nw = wVect.size();
        S_w.reserve(nw);

        for (double w: wVect) {
            S_w.push_back(Eval(w));
        }
        return S_w;
    }

    std::vector<double> FrWaveSpectrum::vectorDiscretization(std::vector<double> vect) {
        assert(!vect.empty());
        std::vector<double> discretization;
        unsigned long Nv = vect.size()-1;
        double dw;
        if (Nv == 0) {
            dw = 1;
            discretization.push_back(dw);
            return discretization;
        }
        dw = vect[1] - vect[0];
        discretization.push_back(dw);
        if (Nv > 2) {
            for (unsigned long iv=1; iv<Nv; iv++) {
                dw = 0.5*(vect[iv+1] - vect[iv-1]);
                discretization.push_back(dw);
            }
        }
        if (Nv > 0) {
            dw = vect[Nv] - vect[Nv - 1];
            discretization.push_back(dw);
        }
        return discretization;
    }

    std::vector<double> FrWaveSpectrum::GetWaveAmplitudes(std::vector<double> waveFrequencies) {
        std::vector<double> wave_ampl;
        auto nbFreq = waveFrequencies.size();
        wave_ampl.reserve(nbFreq);
        // Compute dw
        auto dw = vectorDiscretization(waveFrequencies);
        // Loop on the container to compute the wave amplitudes for the set of wave frequencies
        for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq){
            wave_ampl.push_back(sqrt(2. * Eval(waveFrequencies[ifreq]) * dw[ifreq]));
        }
        return wave_ampl;
    }

    std::vector<std::vector<double>>
    FrWaveSpectrum::GetWaveAmplitudes(std::vector<double> waveFrequencies, std::vector<double> waveDirections) {
        auto nbDir = waveDirections.size();
        auto nbFreq = waveFrequencies.size();

        // Compute the directional function
        auto theta_mean = 0.5*(waveDirections[0] + waveDirections[nbDir-1]);
        std::vector<double> dir_func;
        dir_func.reserve(nbDir);
        if (m_directional_model == nullptr) {dir_func.push_back(1.);}
        else {
            dir_func = m_directional_model->GetSpreadingFunction(waveDirections, theta_mean);
        }
        // Compute dw and dtheta
        auto dtheta = vectorDiscretization(waveDirections);
        auto dw = vectorDiscretization(waveFrequencies);

        // Init the containers
        std::vector<double> wave_ampl_temp;
        wave_ampl_temp.reserve(nbFreq);
        std::vector<std::vector<double>> wave_ampl;
        wave_ampl.reserve(nbDir);

        // Loop on the two containers to compute the wave amplitudes for the sets of wave frequencies and directions
        double test;
        for (unsigned int idir=0; idir<nbDir; ++idir) {
            wave_ampl_temp.clear();
            for (unsigned int ifreq = 0; ifreq < nbFreq; ++ifreq) {
                wave_ampl_temp.push_back(sqrt(2.*Eval(waveFrequencies[ifreq]) * dir_func[idir] * dtheta[idir] * dw[ifreq]));
            }
            wave_ampl.push_back(wave_ampl_temp);
        }
        return wave_ampl;
    }

    std::vector<double>
    FrWaveSpectrum::GetWaveAmplitudes(unsigned int nb_waves, double wmin, double wmax) {

        auto wVect = mathutils::linspace(wmin, wmax, nb_waves);
        double dw = wVect[1] - wVect[0];

        std::vector<double> wave_ampl;
        wave_ampl.reserve(nb_waves);
        for (double w: wVect) {
            wave_ampl.push_back(sqrt(2. * Eval(w) * dw));
        }
        return wave_ampl;
    }

    std::vector<std::vector<double>>
    FrWaveSpectrum::GetWaveAmplitudes(unsigned int nb_waves, double wmin, double wmax,
                                      unsigned int nb_dir, double theta_min, double theta_max, double theta_mean) {

        auto wVect = mathutils::linspace(wmin, wmax, nb_waves);
        auto thetaVect = mathutils::linspace(theta_min, theta_max, nb_dir);
        return GetWaveAmplitudes(wVect,thetaVect);

    }


//    Get the extremal frequency values where the wave energy is concentrated
//    We take extremum values where the power spectral density is equal to 1% of the total variance m0 = hs**2/16
//
//    References
//    ----------
//    Prevosto M., Effect of Directional Spreading and Spectral Bandwidth on the Nonlinearity of the Irregular Waves,
//    Proceedings of the Eighth (1998) International Offshore and Polar Engineering Conference, Montreal, Canada
    void FrWaveSpectrum::GetFrequencyBandwidth(double& wmin, double& wmax) const {
        double m0 = pow(m_significant_height,2)/16.;
        double threshold = m0*0.01;

        wmin = dichotomySearch(1E-4,GetPeakFreq(mathutils::RADS),threshold);
        wmax = dichotomySearch(GetPeakFreq(mathutils::RADS),10.,threshold);
    }

    double FrWaveSpectrum::dichotomySearch(double wmin, double wmax, double threshold) const {

        double wresult, epsilon = 1.E-10;

        while(fabs(wmin-wmax)>epsilon*epsilon)
        {
            wresult=(wmin+wmax)/2.0f;
            if (fabs(Eval(wresult) - threshold) < epsilon)
            {
                return wresult;
            }
            else
            {
                if ((Eval(wmax) - threshold) * (Eval(wresult) - threshold) < 0.0f)
                    wmin=wresult;
                else
                    wmax=wresult;
            }
        }
        throw FrException("FrWaveSpectrum::dichotomySearch no convergence");
    }

    // =================================================================================================================
    // FrJonswapWaveSpectrum descriptions

    FrJonswapWaveSpectrum::FrJonswapWaveSpectrum(double hs, double tp, double gamma) :
            FrWaveSpectrum(hs, tp),
            m_gamma(gamma) {
        CheckGamma();
    }

    void FrJonswapWaveSpectrum::CheckGamma() {
        if (m_gamma < 1. || m_gamma > 10.) {
            // TODO: utiliser un vrai warning sur stderr
            std::cout << "WARNING: Valid values of gamma parameter in JONSWAP wave spectrum are between 1 and 10. " << std::endl;
        }
    }

    double FrJonswapWaveSpectrum::GetGamma() const { return m_gamma; }

    void FrJonswapWaveSpectrum::SetGamma(double gamma) {
        m_gamma = gamma;
        CheckGamma();
    }

    double FrJonswapWaveSpectrum::Eval(double w) const {

        double wp2 = m_peak_frequency * m_peak_frequency;
        double wp4 = wp2 * wp2;

        double w4_1 = std::pow(w, -4.);
        double w5_1 = w4_1 / w;

        double hs2 = m_significant_height * m_significant_height;

        double S_w = 0.;
        if (w > 0.) {

            S_w = 0.3125 * hs2 * wp4 * w5_1 * std::exp(-1.25 * wp4 * w4_1) * (1 - 0.287 * std::log(m_gamma));

            double a = std::exp(-std::pow(w - m_peak_frequency, 2) / (2. * wp2));
            if (w <= m_peak_frequency) {
                a = std::pow(a, _SIGMA2_1_left);
            } else {
                a = std::pow(a, _SIGMA2_1_right);
            }
            S_w *= std::pow(m_gamma, a);
        }

        return S_w;
    }

    // =================================================================================================================
    // FrPiersonMoskowitzWaveSpectrum descriptions
    FrPiersonMoskowitzWaveSpectrum::FrPiersonMoskowitzWaveSpectrum(double hs, double tp) :
            FrWaveSpectrum(hs, tp) {}

    double FrPiersonMoskowitzWaveSpectrum::Eval(double w) const {

        double Tz = mathutils::RADS2S(m_peak_frequency) / 1.408;  // Up-crossing period

        double A = std::pow(MU_2PI/Tz, 4) / (M_PI * std::pow(w, 4));

        double Hs2 = m_significant_height * m_significant_height;

        return 0.25 * A * (Hs2 / w) * std::exp(-A);

    }

    double FrTestWaveSpectrum::Eval(double w) const {
        return 1.;
    }

}  // end namespace frydom
