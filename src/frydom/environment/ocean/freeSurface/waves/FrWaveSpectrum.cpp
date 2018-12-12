//
// Created by frongere on 04/10/17.
//

#include "FrWaveSpectrum.h"

namespace frydom {

    void FrCos2sDirectionalModel::CheckSpreadingFactor() {
        // TODO: utiliser un warning ver la stderr
        if (m_spreading_factor < 1. || m_spreading_factor > 100.) {
            std::cout << "The spreading factor of a cos2s directional spectrum model should lie between 1. and 100." << std::endl;
        }
    }

    FrCos2sDirectionalModel::FrCos2sDirectionalModel(const double spreading_factor) : m_spreading_factor(spreading_factor) {
        CheckSpreadingFactor();
    }

    WaveDirectionalModelType FrCos2sDirectionalModel::GetType() const { return COS2S; }

    double FrCos2sDirectionalModel::GetSpreadingFactor() const { return m_spreading_factor; }

    void FrCos2sDirectionalModel::UpdateSpreadingFunction(const std::vector<double> &thetaVect, const double theta_mean) {
        c_spreading_fcn.clear();
        c_spreading_fcn.reserve(thetaVect.size());

        double s = m_spreading_factor;
        double two_s = 2. * s;

        double c_s = ( pow(2., two_s - 1.) / M_PI ) * pow(std::tgamma(s + 1.), 2.) / std::tgamma(two_s + 1.);

        for (double theta: thetaVect) {
            c_spreading_fcn.push_back(c_s * pow(cos(0.5 * (theta - theta_mean)), two_s));
        }

        c_DTheta = thetaVect[1] - thetaVect[0];
    }

    std::vector<double> FrCos2sDirectionalModel::GetSpreadingFunction() const { return c_spreading_fcn; }

    double FrCos2sDirectionalModel::GetDTheta() const {
        return c_DTheta;
    }

    void FrCos2sDirectionalModel::SetSpreadingFactor(const double spreading_factor) {
        m_spreading_factor = spreading_factor;
        CheckSpreadingFactor();
    }

    std::unique_ptr<FrWaveSpectrum> MakeWaveSpectrum(WAVE_SPECTRUM_TYPE type) {

        switch (type) {

            case JONSWAP:
                return std::make_unique<FrJonswapWaveSpectrum>();

            case PIERSON_MOSKOWITZ:
                return std::make_unique<FrPiersonMoskowitzWaveSpectrum>();

        }

    }

    void FrTestDirectionalModel::UpdateSpreadingFunction(const std::vector<double> &thetaVect, double theta_mean) {
        c_spreading_fcn.clear();
        c_spreading_fcn.reserve(thetaVect.size());
        for (double theta: thetaVect) {
            c_spreading_fcn.push_back(1.);
        }

    }

    WaveDirectionalModelType FrTestDirectionalModel::GetType() const { return DIRTEST; }

    std::vector<double> FrTestDirectionalModel::GetSpreadingFunction() const {return c_spreading_fcn;}

    FrWaveSpectrum::FrWaveSpectrum(const double hs, const double tp, const FREQUENCY_UNIT unit) :
            m_significant_height(hs),
            m_peak_pulsation(convert_frequency(tp, unit, RADS)) {}

    void FrWaveSpectrum::SetCos2sDirectionalModel(double spreadingFactor) {
        m_dir_model_type = COS2S;
        m_directional_model = std::make_unique<FrCos2sDirectionalModel>(spreadingFactor);
    }

    void FrWaveSpectrum::SetDirectionalModel(WaveDirectionalModelType model) {
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

    void FrWaveSpectrum::DirectionalON(WaveDirectionalModelType model) {
        SetDirectionalModel(model);
    }

    void FrWaveSpectrum::DirectionalOFF() {
        m_dir_model_type = NONE;
        m_directional_model = nullptr;
    }

    double FrWaveSpectrum::GetHs() const { return m_significant_height; }

    void FrWaveSpectrum::SetHs(double Hs) { m_significant_height = Hs; }

    double FrWaveSpectrum::GetTp() const { return convert_frequency(m_peak_pulsation, RADS, S); }

    void FrWaveSpectrum::SetTp(double Tp) { m_peak_pulsation = convert_frequency(Tp, S, RADS); }

    double FrWaveSpectrum::GetWp() const { return m_peak_pulsation; }

    void FrWaveSpectrum::SetWp(double Wp) { m_peak_pulsation = Wp; }

    double FrWaveSpectrum::GetFp() const { return convert_frequency(m_peak_pulsation, RADS, HZ); }

    void FrWaveSpectrum::SetFp(double Fp) { m_peak_pulsation = convert_frequency(Fp, HZ, RADS); }

    double FrWaveSpectrum::GetPeakFreq(FREQUENCY_UNIT unit) const {
        return convert_frequency(m_peak_pulsation, RADS, unit);
    }

    std::vector<double> FrWaveSpectrum::Eval(const std::vector<double> wVect) const {
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
            m_directional_model->UpdateSpreadingFunction(waveDirections, theta_mean);
            dir_func = m_directional_model->GetSpreadingFunction();  // TODO: les deux etapes sont inutiles...
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
    FrWaveSpectrum::GetWaveAmplitudes(const unsigned int nb_waves, const double wmin, const double wmax) {

        auto wVect = linspace(wmin, wmax, nb_waves);
        double dw = wVect[1] - wVect[0];

        std::vector<double> wave_ampl;
        wave_ampl.reserve(nb_waves);
        for (double w: wVect) {
            wave_ampl.push_back(sqrt(2. * Eval(w) * dw));
        }
        return wave_ampl;
    }

    std::vector<std::vector<double>>
    FrWaveSpectrum::GetWaveAmplitudes(const unsigned int nb_waves, const double wmin, const double wmax,
                                      const unsigned int nb_dir, const double theta_min, const double theta_max,
                                      const double theta_mean) {

        assert(m_dir_model_type != NONE);

        auto wVect = linspace(wmin, wmax, nb_waves);
        double dw = wVect[1] - wVect[0];

        auto thetaVect = linspace(theta_min, theta_max, nb_dir);
        double dtheta = thetaVect[1] - thetaVect[0];
        m_directional_model->UpdateSpreadingFunction(thetaVect, theta_mean);
        auto dir_func = m_directional_model->GetSpreadingFunction();  // TODO: les deux etapes sont inutiles...

        std::vector<std::vector<double>> S_w_theta;
        S_w_theta.reserve(nb_dir);

        std::vector<double> S_w;
        S_w.reserve(nb_waves);

        for (double f_theta: dir_func) {
            S_w.clear();
            for (double w: wVect) {
                S_w.push_back(sqrt(2. * Eval(w) * f_theta * dtheta * dw));
            }
            S_w_theta.push_back(S_w);
        }
        return S_w_theta;
    }

    FrJonswapWaveSpectrum::FrJonswapWaveSpectrum(const double hs, const double tp, const FREQUENCY_UNIT unit,
                                                 const double gamma) :
            FrWaveSpectrum(hs, tp, unit),
            m_gamma(gamma) {
        CheckGamma();
    }

    void FrJonswapWaveSpectrum::CheckGamma() {
        if (m_gamma < 1. || m_gamma > 10.) {
            // TODO: utiliser un vrai warning sur stderr
            std::cout << "WARNING: Valid values of gamma parameter in Jonswap wave spectrum are between 1 and 10. " << std::endl;
        }
    }

    double FrJonswapWaveSpectrum::GetGamma() const { return m_gamma; }

    void FrJonswapWaveSpectrum::SetGamma(const double gamma) {
        m_gamma = gamma;
        CheckGamma();
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

    FrPiersonMoskowitzWaveSpectrum::FrPiersonMoskowitzWaveSpectrum(const double hs, const double tp,
                                                                   const FREQUENCY_UNIT unit) :
            FrWaveSpectrum(hs, tp, unit) {}

    double FrPiersonMoskowitzWaveSpectrum::Eval(const double w) const {

        double Tz = RADS2S(m_peak_pulsation) / 1.408;  // Up-crossing period

        double A = pow(MU_2PI/Tz, 4) / (M_PI * pow(w, 4));

        double Hs2 = m_significant_height * m_significant_height;

        return 0.25 * A * (Hs2 / w) * exp(-A);

    }

    double FrTestWaveSpectrum::Eval(const double w) const {
        return 1.;
    }
}  // end namespace frydom