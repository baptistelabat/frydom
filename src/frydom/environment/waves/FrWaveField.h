//
// Created by frongere on 31/10/17.
//

#ifndef FRYDOM_FRWAVEFIELD_H
#define FRYDOM_FRWAVEFIELD_H


#include <vector>
#include <complex>
#include <random>

#include "FrWaveSpectrum.h"
//#include "FrWaveProbe.h"
#include "FrWaveDispersionRelation.h"

#define JJ std::complex<double>(0, 1)

namespace frydom {

    class FrWaveField {

    protected:
        double m_time;

    public:
        void Update(const double time) {
            m_time = time;
        }
    };

    // Forward declaration
    class FrLinearWaveProbe;

    class FrLinearWaveField : public FrWaveField {

        enum TYPE {
            REGULAR,
            IRREGULAR,
            DIRECTIONAL
        };

    private:
        TYPE m_type = DIRECTIONAL;
        std::unique_ptr<FrWaveSpectrum> m_waveSpectrum = nullptr;

        double m_minFreq = 0.;
        double m_maxFreq = 2.;
        unsigned int m_nbFreq = 20;

        FrAngleUnit angleUnit = DEG;
        double m_minDir = -180.*M_DEG;
        double m_maxDir = 165.*M_DEG;
        unsigned int m_nbDir = 20;

        double m_meanDir = 0.*M_DEG;

        // For regular wave field
        double m_height = 0.;
        double m_period = 0.;

        std::vector<double> c_waveNumbers;

        std::vector<std::vector<double>> m_wavePhases; // Not used in regular wave field

        std::vector<std::shared_ptr<FrLinearWaveProbe>> m_waveProbes;

    public:

        FrLinearWaveField() : FrWaveField() {
            GenerateRandomWavePhases();
        }

        TYPE GetType() const {
            return m_type;
        }

        void SetType(TYPE type) {
            m_type = type;

            switch (type) {
                case REGULAR:
                    m_waveSpectrum = nullptr;
                    m_nbFreq = 1;
                    m_nbDir = 1;
                    m_minDir = m_meanDir;
                    m_maxDir = m_meanDir;
                    break;
                case IRREGULAR:
                    m_nbDir = 1;
                    m_minDir = m_meanDir;
                    m_maxDir = m_meanDir;
                    break;
                case DIRECTIONAL:
                    
                    break;
            }
        }

        double GetMeanWaveDirection(FrAngleUnit unit=DEG) const {
            if (unit == DEG) {
                return m_meanDir * M_RAD;
            } else {
                return m_meanDir;
            }
        }

        void SetMeanWaveDirection(const double meanDirection, FrAngleUnit unit=DEG) {
            if (unit == DEG) {
                m_meanDir = meanDirection * M_DEG;
            } else {
                m_meanDir = meanDirection;
            }

            if (!m_type == DIRECTIONAL) {
                m_minDir = m_meanDir;
                m_maxDir = m_meanDir;
            }
        }

        std::vector<double> GetWaveDirections(FrAngleUnit unit=DEG) const {
            if (unit == DEG) {
                return linspace(m_minDir*M_RAD, m_maxDir*M_RAD, m_nbDir);
            } else {
                return linspace(m_minDir, m_maxDir, m_nbDir);
            }

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
                    SetType(REGULAR);
                } else {
                    SetType(IRREGULAR);
                }
            }
        }

        std::vector<double> GetWavePulsations(FREQ_UNIT unit=RADS) const {
            return linspace(convert_frequency(m_minFreq, RADS, unit),
                            convert_frequency(m_maxFreq, RADS, unit),
                            m_nbFreq);
        }

        void SetWavePulsations(const double minFreq, const double maxFreq, const unsigned int nbFreq, FREQ_UNIT unit=RADS) {
            m_minFreq = convert_frequency(minFreq, unit, RADS);
            m_maxFreq = convert_frequency(maxFreq, unit, RADS);
            m_nbFreq = nbFreq;

            if (nbFreq == 1) {
                SetType(REGULAR);
            }

            _ComputeWaveNumber();
        }

        void _ComputeWaveNumber() {
            // TODO: aller chercher les infos dans FrOffshoreSystem
            double waterHeight = 10.;
            auto w = GetWavePulsations(RADS);
            double grav = 9.81;
            c_waveNumbers = SolveWaveDispersionRelation(waterHeight, w, grav);
        }

        std::vector<std::vector<double>> GetWavePhases() const {
            return m_wavePhases;
        }

        void SetWavePhases(std::vector<std::vector<double>>& wavePhases) {
            assert(wavePhases.size() == m_nbDir);
            for (auto w: wavePhases) {
                assert(w.size() == m_nbFreq);
            }
            m_wavePhases = wavePhases;
        }

        void GenerateRandomWavePhases() {

            if (m_type == REGULAR) return;

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<double> dis(0., M_2PI);

            m_wavePhases.clear();
            m_wavePhases.reserve(m_nbDir);

            std::vector<double> phases;
            phases.reserve(m_nbFreq);

            for (uint idir=0; idir<m_nbDir; ++idir) {
                phases.clear();
                for (uint iw=0; iw<m_nbFreq; ++iw) {
                    phases.push_back(dis(gen));
                }
                m_wavePhases.push_back(phases);
            }
        }

        std::vector<double> GetWaveLengths() const {
            // TODO
        }

        std::vector<double> GetWaveNumbers() const {
            // TODO
        }

        std::vector<std::vector<double>> GetWaveAmplitudes() const {
            std::vector<std::vector<double>> waveAmplitudes;
            std::vector<double> ampl;
            switch (m_type) {

                case REGULAR:
                    ampl.push_back(m_height * 0.5);
                    waveAmplitudes.push_back(ampl);
                    break;

                case IRREGULAR:
                    ampl = m_waveSpectrum->GetWaveAmplitudes(m_nbFreq, m_minFreq, m_maxFreq);
                    waveAmplitudes.push_back(ampl);
                    break;

                case DIRECTIONAL:
                    waveAmplitudes = m_waveSpectrum->GetWaveAmplitudes(m_nbFreq, m_minFreq, m_maxFreq,
                                                                       m_nbDir, m_minDir, m_maxDir, m_meanDir);
                    break;
            }
            return waveAmplitudes;
        }

        std::vector<std::vector<std::complex<double>>>
        GetCmplxElevation(const double x, const double y, bool steady=false) const {


            std::vector<std::vector<std::complex<double>>> cmplxElevations(m_nbDir);  // is nbDir x nbFreq
            std::vector<std::complex<double>> elev(m_nbFreq);

            std::vector<double> w_(m_nbDir);
            if (m_type == DIRECTIONAL) {
                auto wave_dirs = GetWaveDirections(RAD);
                for (auto wave_dir: wave_dirs) {
                    w_.push_back(x * cos(wave_dir) + y * sin(wave_dir));
                }
            } else {
                w_.push_back(x * cos(m_meanDir) + y * sin(m_meanDir));
            }

            std::vector<std::vector<double>> waveAmplitudes = GetWaveAmplitudes();
            std::vector<double> waveFreqs = GetWavePulsations(RADS);

            std::complex<double> aik, val;
            double ki, wi, wi_, phi_ik;

            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
                elev.clear();

                for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                    aik = waveAmplitudes[idir][ifreq];
                    ki = waveFreqs[ifreq];
                    wi_ = w_[ifreq];
                    phi_ik = m_wavePhases[idir][ifreq];

                    val = aik * exp(JJ * (ki*wi_ + phi_ik));

                    if (!steady) {
                        wi = waveFreqs[ifreq];
                        val *= exp(-JJ*wi*m_time);
                    }

                    elev.push_back(val);
                }
                cmplxElevations.push_back(elev);
            }
            return cmplxElevations;

        }

        std::vector<std::complex<double>> GetSteadyElevation(const double x, const double y) const {
            auto cmplxElevation = GetCmplxElevation(x, y, true);

            std::vector<std::complex<double>> steadyElevation(m_nbFreq);
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

        std::shared_ptr<FrLinearWaveProbe> NewWaveProbe() {
            // TODO
        }

        void Initialize() {
            // TODO: pour initialiser les steady elevation et les tableaux d'elevation
            // TODO: garder le tableau d'elevation (steady) en cache
        }

    };


}  // end namespace frydom

#endif //FRYDOM_FRWAVEFIELD_H
