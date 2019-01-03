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

        double GetFp() const { return convert_frequency(m_peak_pulsation, RADS, HERTZ); }

        void SetFp(double Fp) { m_peak_pulsation = convert_frequency(Fp, HERTZ, RADS); }

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


































    // >>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    class FrWaveDirectionalModel_ {
    public:

        /// Get the type of the directional model
        /// \return type of directional model (COS2S/NONE/DIRTEST)
        virtual WaveDirectionalModelType GetType() const  = 0;

        /// Get the spreading function for a vector of wave directions thetaVect
        /// \param thetaVect vector of wave directions
        /// \param theta_mean mean wave direction
        /// \return spreading function
        virtual std::vector<double> GetSpreadingFunction(const std::vector<double>& thetaVect, double theta_mean);

        /// Evaluate the spreading function for a wave direction
        /// \param theta wave direction
        /// \param theta_mean mean wave direction
        /// \return evaluation of the spreading function
        virtual double Eval(double theta, double theta_mean) const = 0;

        /// Get the extremal frequency values where the power spectral density is equal to 1% of the total variance m0 = hs**2/16
        /// \return extremal frequency values
        void GetDirectionBandwidth(double& theta_min, double& theta_max, double theta_mean) const;

    protected:

        /// Search by dichotomy method.
        /// \param theta_mean mean wave direction
        /// \param threshold threshold
        /// \return
        double dichotomySearch(double theta_mean, double threshold) const;

    };

    // =================================================================================================================
    /// -------------------------------------------------------------------
    /// FrCos2sDirectionalModel
    /// -------------------------------------------------------------------
    /// This directional model, proposed by Longuet-Higgins[1963] is an extension of the cosine-squared model.
    /// the spreading function is given by
    /// spreading_fcn = c_s * cos^2s[(theta-theta0)/2]
    /// where c_s = [ 2^(2s-1)]/Pi . [Gamma²(s+1)]/[Gamma(2s+1)]
    class FrCos2sDirectionalModel_ : public FrWaveDirectionalModel_ {

    private:

        double m_spreading_factor = 10.;    ///< Spreading factor, must be defined between 1. and 100.

        double c_s;                    ///< cached value of [(2^(2s-1))/Pi]*[Gamma²(s+1)]/[Gamma(2s+1)]

        /// Check that the spreading factor is correctly defined between 1. and 100.
        void CheckSpreadingFactor();

        /// Compute the directional spectrum coefficient
        void EvalCs();

    public:

        /// Constructor for the FrCos2sDirectionalModel_
        /// \param spreading_factor spreading factor s
        explicit FrCos2sDirectionalModel_(double spreading_factor=10.);

        /// Get the type of the directional model
        /// \return type of directional model, here COS2S
        WaveDirectionalModelType GetType() const override;

        /// Get the spreading factor s of the cos2s directional model
        /// \return spreading factor s of the cos2s directional model
        double GetSpreadingFactor() const;

        /// Set the spreading factor s of the cos2s directional model
        /// \param spreading_factor spreading factor s of the cos2s directional model
        void SetSpreadingFactor(double spreading_factor);

        /// Evaluate the spreading function for a wave direction
        /// \param theta wave direction
        /// \param theta_mean mean wave direction
        /// \return evaluation of the spreading function
        double Eval(double theta, double theta_mean) const override;

    };

    // =================================================================================================================
    /// For test use only
    class FrTestDirectionalModel_ : public FrWaveDirectionalModel_ {
    public:

        /// Get the type of the directional model
        /// \return type of directional model, here DIRTEST
        WaveDirectionalModelType GetType() const override;

        /// Evaluate the spreading function for a wave direction
        /// \param theta wave direction
        /// \param theta_mean mean wave direction
        /// \return evaluation of the spreading function
        double Eval(double theta, double theta_mean) const override;
    };

    // =================================================================================================================
    /// -------------------------------------------------------------------
    /// FrWaveSpectrum_
    /// -------------------------------------------------------------------
    /// Virtual base class for the wave spectra.
    class FrWaveSpectrum_ : public FrObject {

    protected:
        double m_significant_height = 3.;       ///< Significant height (in meters): mean wave height (trough to crest)
                                                ///<  of the highest third of the waves (H1/3)
        double m_peak_frequency = S2RADS(9.);   ///< Peak circular frequency (in radians/s),
                                                ///< is the wave frequency with the highest energy

        WaveDirectionalModelType m_dir_model_type = NONE;   ///< wave directional model type (NONE/COS2S/DIRTEST)

        std::unique_ptr<FrWaveDirectionalModel_> m_directional_model = nullptr; ///< wave directional model

    public:

        /// Default constructor of the wave spectrum
        FrWaveSpectrum_() = default;

        /// Constructor of the wave spectrum, based on a significant height and peak period, with its associated unit.
        /// \param hs significant height
        /// \param tp peak period
        /// \param unit peak period unit
        FrWaveSpectrum_(double hs, double tp, FREQUENCY_UNIT unit=S);;

        /// Set the wave directional model to a cos2s, with a spreading factor
        /// \param spreadingFactor spreading factor of the cos2s model
        void SetCos2sDirectionalModel(double spreadingFactor);

        /// Set the wave directional model to use from type
        /// \param model wave directional model type
        void SetDirectionalModel(WaveDirectionalModelType model);

        /// Set the wave directional model to use from object
        /// \param dir_model wave directional model
        void SetDirectionalModel(FrWaveDirectionalModel_* dir_model);

        /// Get the wave directional model
        /// \return wave directional model
        FrWaveDirectionalModel_* GetDirectionalModel() const;

        /// Set the wave spectrum as multi-directional
        /// \param model wave directional model type
        void DirectionalON(WaveDirectionalModelType model=COS2S);

        /// Set the wave spectrum as unidirectional
        void DirectionalOFF();

        /// Get the significant height
        /// \return significant height
        double GetHs() const;

        /// Setet the significant height
        /// \param Hs significant height
        void SetHs(double Hs);

        /// Get the peak frequency : period (S), circular frequency (RADS), frequency (HZ), etc.
        /// \param unit unit of the peak frequency (S/RADS/HZ/...)
        /// \return peak frequency
        double GetPeakFreq(FREQUENCY_UNIT unit) const;

        /// Set the peak frequency : period (S), circular frequency (RADS), frequency (HZ), etc.
        /// \param Tp peak frequency
        /// \param unit unit of the peak frequency (S/RADS/HZ/...)
        void SetPeakFreq(double Fp, FREQUENCY_UNIT unit);

        /// Set the significant height and peak frequency
        /// \param Hs significant height
        /// \param Tp peak frequency
        /// \param unit unit of the peak frequency (S/RADS/HZ/...)
        void SetHsTp(double Hs, double Tp, FREQUENCY_UNIT unit);

        /// Get the extremal frequency values where the power spectral density is equal to 1% of the total variance m0 = hs**2/16
        /// \return extremal frequency values
        void GetFrequencyBandwidth(double& wmin, double& wmax) const;

        /// Eval the spectrum at one frequency
        /// Must be implemented into each wave spectrum
        /// \param w circular frequency for which the wave spectrum is evaluated
        /// \return evaluation of the wave spectrum
        virtual double Eval(double w) const = 0;

        /// Eval the spectrum at a vector of frequencies
        /// \param wVect vector of circular frequencies for which the wave spectrum is evaluated
        /// \return evaluation of the wave spectrum
        std::vector<double> Eval(const std::vector<double> &wVect) const;

        /// Get the discretisation of a vector, using first order centered finite difference scheme
        /// dv(i) = [v(i+1)-v(i-1)]/2
        /// \param vect vector to be discretized
        /// \return discretisation of a vector
        std::vector<double> vectorDiscretization(std::vector<double> vect);

        /// Get the wave amplitudes for a given regular frequency discretization
        /// \param waveFrequencies vector of circular frequencies
        /// \return wave amplitude for a given regular frequency discretization
        virtual std::vector<double> GetWaveAmplitudes(std::vector<double> waveFrequencies);


        /// Get the wave amplitudes for a given frequency and direction discretizations
        /// \param waveFrequencies vector of circular frequencies
        /// \param waveDirections vector of circular directions
        /// \return wave amplitudes for a given frequency and direction discretizations
        virtual std::vector<std::vector<double>> GetWaveAmplitudes(std::vector<double> waveFrequencies, std::vector<double> waveDirections);


        /// Get the wave amplitudes for a given regular frequency discretization
        /// \param nb_waves number of wave discretization
        /// \param wmin minimum circular frequency
        /// \param wmax maximum circular frequency
        /// \return wave amplitudes for a given regular frequency discretization
        virtual std::vector<double> GetWaveAmplitudes(unsigned int nb_waves, double wmin, double wmax);

        /// Get the wave amplitudes for a given regular frequency and direction discretizations
        /// \param nb_waves number of wave freuencies
        /// \param wmin minimum circular frequency
        /// \param wmax maximum circular frequency
        /// \param nb_dir number of wave directions
        /// \param theta_min minimum direction
        /// \param theta_max maximum direction
        /// \param theta_mean mean direction
        /// \return wave amplitudes for a given regular frequency and direction discretizations
        virtual std::vector<std::vector<double>> GetWaveAmplitudes(unsigned int nb_waves, double wmin, double wmax,
                unsigned int nb_dir, double theta_min, double theta_max, double theta_mean);

        /// Initialize the state of the wave spectrum
        void Initialize() override {}

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override {}

    private:

        /// Search by dichotomy method.
        /// \param wmin
        /// \param wmax
        /// \return
        double dichotomySearch(double wmin, double wmax, double threshold) const;

    };

    // =================================================================================================================
    // FIXME: est-ce vraiment critique que d'avoir les 2 choses suivantes en macro ???
//    #define _SIGMA2_1_left (1./(0.07*0.07))
//    #define _SIGMA2_1_right (1./(0.09*0.09))
    /// -------------------------------------------------------------------
    /// FrJonswapWaveSpectrum_
    /// -------------------------------------------------------------------
    /// Class for a Jonswap wave spectrum
    ///
    ///    References
    ///    ----------
    ///    Kim C.H., Nonlinear Waves and Offshore structures, 2008
    ///    Molin B., Hydrodynamique des Structures Offshore, 2002
    ///
    class FrJonswapWaveSpectrum_ : public FrWaveSpectrum_ {

    private:
        double m_gamma = 3.3;   ///< Peakedness factor of the Jonswap wave spectrum,
                                ///< it is the ratio of the maximum of JONSWAP wave spectral density
                                ///< to the maximum of Pierson-Moskowitz spectral density.

    public:

        /// Default constructor
        FrJonswapWaveSpectrum_() = default;

        /// Constructor for a Jonswap wave spectrum, based on the significant height, peak frequency and its associated unit
        /// and a gamma factor
        /// \param hs significant height
        /// \param tp peak frequency
        /// \param unit peak frequency unit
        /// \param gamma gamma factor of the Jonswap spectrum
        FrJonswapWaveSpectrum_(double hs, double tp, FREQUENCY_UNIT unit=S, double gamma=3.3);

        /// Check that the gamma factor is correctly defined between 1. and 10.
        void CheckGamma();

        /// Get the gamma factor of the Jonswap spectrum
        /// \return gamma factor of the Jonswap spectrum
        double GetGamma() const;

        /// Get the gamma factor of the Jonswap spectrum
        /// \param gamma gamma factor of the Jonswap spectrum
        void SetGamma(double gamma);

        /// Eval the spectrum at one frequency
        /// \param w circular frequency for which the wave spectrum is evaluated
        /// \return evaluation of the wave spectrum
        double Eval(double w) const final;

    };


    // =================================================================================================================
    /// -------------------------------------------------------------------
    /// FrPiersonMoskowitzWaveSpectrum_
    /// -------------------------------------------------------------------
    /// Class for a Pierson Moskowitz wave spectrum
    class FrPiersonMoskowitzWaveSpectrum_ : public FrWaveSpectrum_ {

    public:

        /// Default constructor of a Pierson Moskowitz wave spectrum
        FrPiersonMoskowitzWaveSpectrum_() = default;

        /// Constructor for a Pierson Moskowitz wave spectrum, based on the significant height, peak frequency
        /// and its associated unit
        /// \param hs significant height
        /// \param tp peak frequency
        /// \param unit peak frequency unit
        FrPiersonMoskowitzWaveSpectrum_(double hs, double tp, FREQUENCY_UNIT unit=S);

        /// Eval the spectrum at one frequency
        /// \param w circular frequency for which the wave spectrum is evaluated
        /// \return evaluation of the wave spectrum
        double Eval(double w) const final;

    };


    // =================================================================================================================
    /// For test use only
    class FrTestWaveSpectrum_ : public FrWaveSpectrum_ {
    public:
        FrTestWaveSpectrum_() = default;
        double Eval(double w) const final;
        
    };


}  // end namespace frydom


#endif //FRYDOM_FRWAVESPECTRUM_H
