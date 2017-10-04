//
// Created by frongere on 04/10/17.
//

#ifndef FRYDOM_FRWAVESPECTRUM_H
#define FRYDOM_FRWAVESPECTRUM_H

#include <vector>

#include "frydom/core/FrConstants.h"
#include "frydom/misc/FrLinspace.h"

namespace frydom {

    class GenericWaveSpectrum {};

    class FrWaveSpectrum : public GenericWaveSpectrum {

    private:
        double m_significant_height;
        double m_peak_pulsation;

    public:

        FrWaveSpectrum(const double hs, const double tp, const FREQ_UNIT unit=S) :
                m_significant_height(hs),
                m_peak_pulsation(convert_frequency(tp, unit, RADS)) {}

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

    #define _SIGMA2_1_left 1/(0.07*0.07)
    #define _SIGMA2_1_right 1/(0.09*0.09)

    class FrJonswapWaveSpectrum : public FrWaveSpectrum {

    private:
        double m_gamma;

    public:

        FrJonswapWaveSpectrum(const double hs, const double tp, const FREQ_UNIT unit=S, const double gamma=3.3) :
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

//        double Eval(const double w, FREQ_UNIT unit=RADS) {
//
//
//        }




    };


}  // end namespace frydom


#endif //FRYDOM_FRWAVESPECTRUM_H
