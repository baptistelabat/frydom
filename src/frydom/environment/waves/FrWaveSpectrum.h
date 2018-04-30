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
        COS2S
    };

    // =================================================================================================================

    class FrWaveDirectionalModel {
    public:

        virtual WaveDirectionalModelType GetType() const  = 0;

        virtual void UpdateSpreadingFunction(const unsigned int nb_dir,
                                             const double theta_min,
                                             const double theta_max,
                                             const double theta_mean) = 0;

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

        void CheckSpreadingFactor() {
            // TODO: utiliser un warning ver la stderr
            if (m_spreading_factor < 1. || m_spreading_factor > 100.) {
                std::cout << "The spreading factor of a cos2s directional spectrum model should lie between 1. and 100." << std::endl;
            }
        }

    public:

        explicit FrCos2sDirectionalModel(const double spreading_factor=10.) : m_spreading_factor(spreading_factor) {
            CheckSpreadingFactor();
        }

        WaveDirectionalModelType GetType() const override { return COS2S; }

        double GetSpreadingFactor() const { return m_spreading_factor; }

        void SetSpreadingFactor(const double spreading_factor) {
            m_spreading_factor = spreading_factor;
            CheckSpreadingFactor();
        }

        void UpdateSpreadingFunction(const unsigned int nb_dir,
                                     const double theta_min,
                                     const double theta_max,
                                     const double theta_mean) override {

            UpdateSpreadingFunction(linspace(theta_min, theta_max, nb_dir), theta_mean);
        }

        void UpdateSpreadingFunction(const std::vector<double>& thetaVect, const double theta_mean) override {
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

        std::vector<double> GetSpreadingFunction() const override { return c_spreading_fcn; }

        double GetDTheta() const {
            return c_DTheta;
        }

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
        FrWaveSpectrum(const double hs, const double tp, const FREQUENCY_UNIT unit=S) :
                m_significant_height(hs),
                m_peak_pulsation(convert_frequency(tp, unit, RADS)) {}

        /// Set the directional model to use from type
        void SetDirectionalModel(WaveDirectionalModelType model) {
            switch (model) {
                case NONE:
                    DirectionalOFF();
                    break;
                case COS2S:
                    m_dir_model_type = COS2S;
                    m_directional_model = std::make_unique<FrCos2sDirectionalModel>();
                    break;
            }
        }

        /// Set the directional model to use from objetct
        void SetDirectionalModel(FrWaveDirectionalModel* dir_model) {
            m_dir_model_type = dir_model->GetType();
            m_directional_model = std::unique_ptr<FrWaveDirectionalModel>(dir_model);
        }

        FrWaveDirectionalModel* GetDirectionalModel() const {
            return m_directional_model.get();
        }

        void DirectionalON(WaveDirectionalModelType model=COS2S) {
            SetDirectionalModel(model);
        }

        void DirectionalOFF() {
            m_dir_model_type = NONE;
            m_directional_model = nullptr;
        }

        double GetHs() const { return m_significant_height; }

        void SetHs(double Hs) { m_significant_height = Hs; }

        double GetTp() const { return convert_frequency(m_peak_pulsation, RADS, S); }

        void SetTp(double Tp) { m_peak_pulsation = convert_frequency(Tp, S, RADS); }

        double GetWp() const { return m_peak_pulsation; }

        void SetWp(double Wp) { m_peak_pulsation = Wp; }

        double GetFp() const { return convert_frequency(m_peak_pulsation, RADS, HZ); }

        void SetFp(double Fp) { m_peak_pulsation = convert_frequency(Fp, HZ, RADS); }

        double GetPeakFreq(FREQUENCY_UNIT unit) const {
            return convert_frequency(m_peak_pulsation, RADS, unit);
        }

        void GetFrequencyBandwidth(double& wmin, double& wmax) const {
            // TODO
        }

        /// Eval the spectrum at one frequency
        /// Must be implemented into each wave spectrum
        virtual double Eval(const double w) const = 0;

        /// Eval the spectrum at a vector of frequencies
        virtual std::vector<double> Eval(const std::vector<double> wVect) const {
            std::vector<double> S_w;
            auto nw = wVect.size();
            S_w.reserve(nw);

            for (double w: wVect) {
                S_w.push_back(Eval(w));
            }
            return S_w;
        }


        /// Get the wave amplitudes for a given regular frequency discretization
        virtual std::vector<double> GetWaveAmplitudes(const unsigned int nb_waves,
                                                      const double wmin,
                                                      const double wmax) {

            auto wVect = linspace(wmin, wmax, nb_waves);
            double dw = wVect[1] - wVect[0];

            std::vector<double> wave_ampl;
            wave_ampl.reserve(nb_waves);
            for (double w: wVect) {
                wave_ampl.push_back(sqrt(2. * Eval(w) * dw));
            }
            return wave_ampl;
        }

        virtual std::vector<std::vector<double>> GetWaveAmplitudes(const unsigned int nb_waves,
                                                                   const double wmin,
                                                                   const double wmax,
                                                                   const unsigned int nb_dir,
                                                                   const double theta_min,
                                                                   const double theta_max,
                                                                   const double theta_mean) {

            if (m_dir_model_type == NONE) {
                // ERROR !!
            }

            auto wVect = linspace(wmin, wmax, nb_waves);
            double dw = wVect[1] - wVect[0];

            auto thetaVect = linspace(wmin, wmax, nb_dir);
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

        virtual void Initialize() override {}

        virtual void StepFinalize() override {}

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

        FrJonswapWaveSpectrum(const double hs, const double tp, const FREQUENCY_UNIT unit=S, const double gamma=3.3) :
                FrWaveSpectrum(hs, tp, unit),
                m_gamma(gamma) {
            CheckGamma();
        }

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

        double Eval(const double w) const final {

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

    };

    // =================================================================================================================

    class FrPiersonMoskowitzWaveSpectrum : public FrWaveSpectrum {

    public:

        FrPiersonMoskowitzWaveSpectrum() = default;

        FrPiersonMoskowitzWaveSpectrum(const double hs, const double tp, const FREQUENCY_UNIT unit=S) :
                FrWaveSpectrum(hs, tp, unit) {}

        double Eval(const double w) const final {

            double Tz = RADS2S(m_peak_pulsation) / 1.408;  // Up-crossing period

            double A = pow(MU_2PI/Tz, 4) / (M_PI * pow(w, 4));

            double Hs2 = m_significant_height * m_significant_height;

            return 0.25 * A * (Hs2 / w) * exp(-A);

        }

    };


    std::unique_ptr<FrWaveSpectrum> MakeWaveSpectrum(WAVE_SPECTRUM_TYPE type);



}  // end namespace frydom


#endif //FRYDOM_FRWAVESPECTRUM_H
