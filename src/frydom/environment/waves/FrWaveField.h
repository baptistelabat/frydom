//
// Created by frongere on 31/10/17.
//

#ifndef FRYDOM_FRWAVEFIELD_H
#define FRYDOM_FRWAVEFIELD_H


#include <vector>
#include <complex>
#include <random>

#include "FrWaveSpectrum.h"
#include "FrWaveDispersionRelation.h"

#define JJ std::complex<double>(0, 1)

namespace frydom {

    // Forward declarations
    class FrWaveProbe;
    class FrLinearWaveProbe;



    enum WAVE_MODEL {
        NO_WAVES,
        LINEAR_WAVES
    };

    class FrWaveField {

    protected:
        static const WAVE_MODEL m_waveModel;

        double m_time;

        std::unique_ptr<FrWaveSpectrum> m_waveSpectrum = nullptr;

    public:

        virtual void Update(double time) {
            m_time = time;
        }

        void SetWaveSpectrum(WAVE_SPECTRUM_TYPE type) {
            m_waveSpectrum = MakeWaveSpectrum(type);
        }

        WAVE_MODEL GetWaveModel() const { return m_waveModel; }

        virtual double GetElevation(double x, double y) const = 0;

        virtual std::vector<std::vector<double>> GetElevation(const std::vector<double>& xVect,
                                                              const std::vector<double>& yVect) const = 0;

//        virtual std::shared_ptr<FrWaveProbe> NewWaveProbe(double x, double y) = 0;

    };


    class FrNullWaveField : public FrWaveField {

    private:
        static const WAVE_MODEL m_waveModel = NO_WAVES;


    public:
        FrNullWaveField() {}

        double GetElevation(double x, double y) const final {
            return 0.;
        }

        std::vector<std::vector<double>> GetElevation(const std::vector<double>& xVect,
                                                      const std::vector<double>& yVect) const final {

            auto nx = xVect.size();
            auto ny = yVect.size();

            std::vector<std::vector<double>> elevations;
            elevations.reserve(nx);

            std::vector<double> elev;
            elev.reserve(ny);
            for (unsigned int iy=0; iy<ny; ++iy) {
                elev.push_back(0.);
            }

            for (unsigned int ix=0; ix<nx; ++ix) {
                elevations.push_back(elev);
            }

            return elevations;

        }

        std::shared_ptr<FrWaveProbe> NewWaveProbe(double x, double y) {
            // TODO
        }

    };

    enum LINEAR_WAVE_TYPE {
        LINEAR_REGULAR,
        LINEAR_IRREGULAR,
        LINEAR_DIRECTIONAL
    };

    // =================================================================================================================

    class FrLinearWaveField : public FrWaveField {

    private:
        static const WAVE_MODEL m_waveModel = LINEAR_WAVES;

        LINEAR_WAVE_TYPE m_linearWaveType = LINEAR_DIRECTIONAL;

        double m_minFreq = 0.;
        double m_maxFreq = 2.;
        unsigned int m_nbFreq = 40;

        double m_minDir = -180.*M_DEG;
        double m_maxDir = 165.*M_DEG;
        unsigned int m_nbDir = 20;

        double m_meanDir = 0.*M_DEG;

        // For regular wave field
        double m_height = 0.;
        double m_period = 0.;

        std::vector<double> c_waveFrequencies;
        std::vector<double> c_waveNumbers;
        std::vector<std::complex<double>> c_emjwt;

        std::vector<std::vector<double>> m_wavePhases; // Not used in regular wave field

        std::vector<std::shared_ptr<FrLinearWaveProbe>> m_waveProbes;

    public:

        explicit FrLinearWaveField(LINEAR_WAVE_TYPE type) {
            SetType(type);
            GenerateRandomWavePhases();
            Initialize();
            Update(0.);

        }

        LINEAR_WAVE_TYPE GetType() const {
            return m_linearWaveType;
        }

        void SetType(LINEAR_WAVE_TYPE type) {
            m_linearWaveType = type;

            switch (type) {
                case LINEAR_REGULAR:
                    m_waveSpectrum = nullptr;
                    m_nbFreq = 1;
                    m_nbDir = 1;
                    m_minDir = m_meanDir;
                    m_maxDir = m_meanDir;
                    break;
                case LINEAR_IRREGULAR:
                    m_nbDir = 1;
                    m_minDir = m_meanDir;
                    m_maxDir = m_meanDir;

                    // Creating a default waveSpectrum
                    SetWaveSpectrum(JONSWAP);
                    break;
                case LINEAR_DIRECTIONAL:
                    
                    break;
            }
        }

        void SetRegularWaveHeight(double height) {
            m_height = height;
        }

        void SetRegularWavePeriod(double period, FREQ_UNIT unit=S) {
            m_period = convert_frequency(period, unit, S);
        }

        unsigned int GetNbFrequencies() const { return m_nbFreq; }

        double GetMinFrequency() const { return m_minFreq; }

        double GetMaxFrequency() const { return m_maxFreq; }

        unsigned int GetNbWaveDirections() const { return m_nbDir; }

        double GetMinWaveDirection() const { return m_minDir; }

        double GetMaxWaveDirection() const { return m_maxDir; }

        double GetMeanWaveDirection(FrAngleUnit unit=DEG) const {
            double meanWaveDir = m_meanDir;
            if (unit == DEG) {
                meanWaveDir *= M_RAD;
            }
            return meanWaveDir;
        }

        void SetMeanWaveDirection(const double meanDirection, FrAngleUnit unit=DEG) {
            if (unit == DEG) {
                m_meanDir = meanDirection * M_DEG;
            } else {
                m_meanDir = meanDirection;
            }

            if (!(m_linearWaveType == LINEAR_DIRECTIONAL)) {
                m_minDir = m_meanDir;
                m_maxDir = m_meanDir;
            }
        }

        std::vector<double> GetWaveDirections(FrAngleUnit unit=DEG) const {
            std::vector<double> waveDirections;

            if (m_linearWaveType != LINEAR_DIRECTIONAL) {
                waveDirections.push_back(GetMeanWaveDirection(unit));

            } else {
                if (unit == DEG) {
                    waveDirections = linspace(m_minDir * M_RAD, m_maxDir * M_RAD, m_nbDir);
                } else {
                    waveDirections = linspace(m_minDir, m_maxDir, m_nbDir);
                }

            }
            return waveDirections;
        }

        void SetWaveDirections(const double minDir, const double maxDir, const unsigned int nbDir, FrAngleUnit unit=DEG) {
            if (unit == DEG) {
                m_minDir = minDir * M_DEG;
                m_maxDir = maxDir * M_DEG;
            } else {
                m_minDir = minDir;
                m_maxDir = maxDir;
            }
            m_nbDir = nbDir;

            if (m_nbDir == 1) {
                if (m_nbFreq == 1) {
                    SetType(LINEAR_REGULAR);
                } else {
                    SetType(LINEAR_IRREGULAR);
                }
            }
        }

        std::vector<double> GetWaveFrequencies(FREQ_UNIT unit = RADS) const { return c_waveFrequencies; }

        void SetWavePulsations(const double minFreq, const double maxFreq, const unsigned int nbFreq, FREQ_UNIT unit=RADS) {
            m_minFreq = convert_frequency(minFreq, unit, RADS);
            m_maxFreq = convert_frequency(maxFreq, unit, RADS);
            m_nbFreq = nbFreq;

            if (nbFreq == 1) {
                SetType(LINEAR_REGULAR);
            }

            Initialize();
        }

        void Initialize() {
            // TODO: aller chercher les infos dans FrOffshoreSystem

            // Caching wave frequencies
            c_waveFrequencies.clear();

            if (m_linearWaveType == LINEAR_REGULAR) {
                c_waveFrequencies.push_back(S2RADS(m_period));
            } else {
                c_waveFrequencies = linspace(m_minFreq, m_maxFreq, m_nbFreq);
            }

            // Caching wave numbers
            c_waveNumbers.clear();

            // waterHeight = environment->GetDepth()
            // grav = environment->GetGrav()
            // TODO: charger le hauteur d'eau et la gravite depuis
            double waterHeight = 10.;
            double grav = 9.81;
            c_waveNumbers = SolveWaveDispersionRelation(waterHeight, c_waveFrequencies, grav);
        }

        std::vector<std::vector<double>> GetWavePhases() const {
            return m_wavePhases;
        }

        void SetWavePhases(std::vector<std::vector<double>>& wavePhases) {
            assert(wavePhases.size() == m_nbDir);
            for (auto& w: wavePhases) {
                assert(w.size() == m_nbFreq);
            }
            m_wavePhases = wavePhases;
        }

        void GenerateRandomWavePhases() {

            m_wavePhases.clear();
            m_wavePhases.reserve(m_nbDir);

            std::vector<double> phases;
            phases.reserve(m_nbFreq);

            if (m_linearWaveType == LINEAR_REGULAR) {
                phases.push_back(0.);
                m_wavePhases.push_back(phases);
            } else {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<double> dis(0., M_2PI);

                for (uint idir=0; idir<m_nbDir; ++idir) {
                    phases.clear();
                    for (uint iw=0; iw<m_nbFreq; ++iw) {
                        phases.push_back(dis(gen));
                    }
                    m_wavePhases.push_back(phases);
                }
            }
        }

        std::vector<double> GetWaveLengths() const {
            // TODO
        }

        std::vector<double> GetWaveNumbers() const {
            return c_waveNumbers;
        }

        std::vector<std::vector<double>> _GetWaveAmplitudes() const;

        std::vector<std::vector<std::complex<double>>>
        GetCmplxElevation(const double x, const double y, bool steady=false) const {


            std::vector<std::vector<std::complex<double>>> cmplxElevations;  // is nbDir x nbFreq
            cmplxElevations.reserve(m_nbDir);
            std::vector<std::complex<double>> elev;
            elev.reserve(m_nbFreq);

            std::vector<double> w_;
            w_.reserve(m_nbDir);
            if (m_linearWaveType == LINEAR_DIRECTIONAL) {
                auto wave_dirs = GetWaveDirections(RAD);
                for (auto wave_dir: wave_dirs) {
                    w_.push_back(x * cos(wave_dir) + y * sin(wave_dir));
                }
            } else {
                w_.push_back(x * cos(m_meanDir) + y * sin(m_meanDir));
            }

            std::vector<std::vector<double>> waveAmplitudes = _GetWaveAmplitudes();

            std::complex<double> aik, val;
            double ki, wi, wk_, phi_ik;

            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
                elev.clear();
                wk_ = w_[idir];

                for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                    aik = waveAmplitudes[idir][ifreq];
                    ki = c_waveNumbers[ifreq];  // FIXME: Ici, on doit avoir le nombre d'onde
                    phi_ik = m_wavePhases[idir][ifreq];

                    val = aik * exp(JJ * (ki*wk_ + phi_ik));

                    if (!steady) {  // TODO : utiliser les valeurs mises en cache...
                        val *= c_emjwt[ifreq];
                    }

                    elev.push_back(val);
                }
                cmplxElevations.push_back(elev);
            }
            return cmplxElevations;

        }

        std::vector<std::complex<double>> GetSteadyElevation(const double x, const double y) const {
            auto cmplxElevation = GetCmplxElevation(x, y, true);

            std::vector<std::complex<double>> steadyElevation;
            steadyElevation.reserve(m_nbFreq);
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                steadyElevation.emplace_back(0.);
            }

            // TODO: continuer 
            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
                for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                    steadyElevation[ifreq] += cmplxElevation[idir][ifreq];
                }
            }
            return steadyElevation;

        }

        FrWaveSpectrum* GetWaveSpectrum() const{ return m_waveSpectrum.get(); }

        void SetReturnPeriod() {
            // TODO
        }

        double GetReturnPeriod() const {
            // TODO
        }

        std::shared_ptr<FrLinearWaveProbe> NewWaveProbe(double x, double y);

        void Update(double time) override {
            m_time = time;

            // Updating exp(-jwt)
            std::vector<double> w = GetWaveFrequencies(RADS);
            c_emjwt.clear();
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                c_emjwt.push_back(exp(-JJ*w[ifreq]*time));
            }

        }

        // TODO: renvoyer un pointeur partagé ??
        const std::vector<std::complex<double>>& GetTimeCoeffs() const {
            return c_emjwt;
        }

        double GetElevation(double x, double y) const {

            auto steadyElevation =  GetSteadyElevation(x, y);

            double elevation = 0.;
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                elevation += std::imag( steadyElevation[ifreq] * c_emjwt[ifreq] );
            }
            return elevation;
        }

        std::vector<std::vector<double>> GetElevation(const std::vector<double>& xVect,
                                                      const std::vector<double>& yVect) const {

            auto nx = xVect.size();
            auto ny = yVect.size();

            std::vector<std::vector<double>> elevations;
            std::vector<double> elev;

            elevations.reserve(nx);
            elev.reserve(ny);

            double eta, x, y;
            for (unsigned int ix=0; ix<nx; ++ix) {
                elev.clear();
                x = xVect[ix];
                for (unsigned int iy=0; iy<ny; ++iy) {
                    y = yVect[iy];
                    eta = GetElevation(x, y);
                    elev.push_back(eta);
                }
                elevations.push_back(elev);
            }
            return elevations;
        }


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


}  // end namespace frydom

#endif //FRYDOM_FRWAVEFIELD_H
