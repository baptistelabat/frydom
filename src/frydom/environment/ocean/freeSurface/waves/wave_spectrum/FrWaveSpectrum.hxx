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


//#include "FrWaveSpectrum.h"

//#include "MathUtils/VectorGeneration.h"

namespace frydom {

    // =================================================================================================================
    // FrWaveSpectrum descriptions

    template<class OffshoreSystemType, class DirectionalModel>
    FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::FrWaveSpectrum(double hs, double tp) :
        m_significant_height(hs),
        m_peak_frequency(convert_frequency(tp, mathutils::S, mathutils::RADS)) {
      m_directional_model = std::make_unique<OffshoreSystemType, DirectionalModel>();
    }

//    void FrWaveSpectrum::SetCos2sDirectionalModel(double spreadingFactor) {
//        m_dir_model_type = COS2S;
//        m_directional_model = std::make_unique<FrCos2sDirectionalModel>(spreadingFactor);
//    }
//
//    void FrWaveSpectrum::SetDirectionalModel(WAVE_DIRECTIONAL_MODEL model) {
//        switch (model) {
//            case NONE:
//                DirectionalOFF();
//                break;
//            case COS2S:
//                m_dir_model_type = COS2S;
//                m_directional_model = std::make_unique<FrCos2sDirectionalModel>();
//                break;
//            case DIRTEST:
//                m_dir_model_type = DIRTEST;
//                m_directional_model = std::make_unique<FrTestDirectionalModel>();
//        }
//    }
//
//    void FrWaveSpectrum::SetDirectionalModel(FrWaveDirectionalModel *dir_model) {
//        m_dir_model_type = dir_model->GetType();
//        m_directional_model = std::unique_ptr<FrWaveDirectionalModel>(dir_model);
//    }


    template<class OffshoreSystemType, class DirectionalModel>
    DirectionalModel *FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::GetDirectionalModel() const {
      return m_directional_model.get();
    }

//    void FrWaveSpectrum::DirectionalON(WAVE_DIRECTIONAL_MODEL model) {
//        SetDirectionalModel(model);
//    }
//
//    void FrWaveSpectrum::DirectionalOFF() {
//        m_dir_model_type = NONE;
//        m_directional_model = nullptr;
//    }


    template<class OffshoreSystemType, class DirectionalModel>
    double FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::GetHs() const { return m_significant_height; }

    template<class OffshoreSystemType, class DirectionalModel>
    void FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::SetHs(double Hs) { m_significant_height = Hs; }

    template<class OffshoreSystemType, class DirectionalModel>
    double FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::GetPeakFreq(FREQUENCY_UNIT unit) const {
      return convert_frequency(m_peak_frequency, mathutils::RADS, unit);
    }

    template<class OffshoreSystemType, class DirectionalModel>
    void FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::SetPeakFreq(double Fp, FREQUENCY_UNIT unit) {
      m_peak_frequency = convert_frequency(Fp, unit, mathutils::RADS);
    }

    template<class OffshoreSystemType, class DirectionalModel>
    void FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::SetHsTp(double Hs, double Tp, FREQUENCY_UNIT unit) {
      SetHs(Hs);
      SetPeakFreq(Tp, unit);
    }

    template<class OffshoreSystemType, class DirectionalModel>
    std::vector<double>
    FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::Eval(const std::vector<double> &wVect) const {
      std::vector<double> S_w;
      auto nw = wVect.size();
      S_w.reserve(nw);

      for (double w: wVect) {
        S_w.push_back(Eval(w));
      }
      return S_w;
    }

    template<class OffshoreSystemType, class DirectionalModel>
    std::vector<double>
    FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::vectorDiscretization(std::vector<double> vect) {
      assert(!vect.empty());
      std::vector<double> discretization;
      unsigned long Nv = vect.size() - 1;
      double dw;
      if (Nv == 0) {
        dw = 1;
        discretization.push_back(dw);
        return discretization;
      }
      dw = vect[1] - vect[0];
      discretization.push_back(dw);
      if (Nv > 2) {
        for (unsigned long iv = 1; iv < Nv; iv++) {
          dw = 0.5 * (vect[iv + 1] - vect[iv - 1]);
          discretization.push_back(dw);
        }
      }
      if (Nv > 0) {
        dw = vect[Nv] - vect[Nv - 1];
        discretization.push_back(dw);
      }
      return discretization;
    }

    template<class OffshoreSystemType, class DirectionalModel>
    std::vector<double>
    FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::GetWaveAmplitudes(std::vector<double> waveFrequencies) {
      std::vector<double> wave_ampl;
      auto nbFreq = waveFrequencies.size();
      wave_ampl.reserve(nbFreq);
      // Compute dw
      auto dw = vectorDiscretization(waveFrequencies);
      // Loop on the container to compute the wave amplitudes for the set of wave frequencies
      for (unsigned int ifreq = 0; ifreq < nbFreq; ++ifreq) {
        wave_ampl.push_back(std::sqrt(2. * Eval(waveFrequencies[ifreq]) * dw[ifreq]));
      }
      return wave_ampl;
    }

    template<class OffshoreSystemType, class DirectionalModel>
    std::vector<std::vector<double>>
    FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::GetWaveAmplitudes(std::vector<double> waveFrequencies,
                                                                            std::vector<double> waveDirections) {
      auto nbDir = waveDirections.size();
      auto nbFreq = waveFrequencies.size();

      // Compute the directional function
      auto theta_mean = 0.5 * (waveDirections[0] + waveDirections[nbDir - 1]);
      std::vector<double> dir_func;
      dir_func.reserve(nbDir);
      if (m_directional_model == nullptr) { dir_func.push_back(1.); }
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
      for (unsigned int idir = 0; idir < nbDir; ++idir) {
        wave_ampl_temp.clear();
        for (unsigned int ifreq = 0; ifreq < nbFreq; ++ifreq) {
          wave_ampl_temp.push_back(
              std::sqrt(2. * Eval(waveFrequencies[ifreq]) * dir_func[idir] * dtheta[idir] * dw[ifreq]));
        }
        wave_ampl.push_back(wave_ampl_temp);
      }
      return wave_ampl;
    }

    template<class OffshoreSystemType, class DirectionalModel>
    std::vector<double>
    FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::GetWaveAmplitudes(unsigned int nb_waves, double wmin,
                                                                            double wmax) {

      auto wVect = mathutils::linspace(wmin, wmax, nb_waves);
      double dw = wVect[1] - wVect[0];

      std::vector<double> wave_ampl;
      wave_ampl.reserve(nb_waves);
      for (double w: wVect) {
        wave_ampl.push_back(std::sqrt(2. * Eval(w) * dw));
      }
      return wave_ampl;
    }

    template<class OffshoreSystemType, class DirectionalModel>
    std::vector<std::vector<double>>
    FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::GetWaveAmplitudes(unsigned int nb_waves, double wmin,
                                                                            double wmax,
                                                                            unsigned int nb_dir, double theta_min,
                                                                            double theta_max,
                                                                            double theta_mean) {

      auto wVect = mathutils::linspace(wmin, wmax, nb_waves);
      auto thetaVect = mathutils::linspace(theta_min, theta_max, nb_dir);
      return GetWaveAmplitudes(wVect, thetaVect);

    }


//    Get the extremal frequency values where the wave energy is concentrated
//    We take extremum values where the power spectral density is equal to 1% of the total variance m0 = hs**2/16
//
//    References
//    ----------
//    Prevosto M., Effect of Directional Spreading and Spectral Bandwidth on the Nonlinearity of the Irregular Waves,
//    Proceedings of the Eighth (1998) International Offshore and Polar Engineering Conference, Montreal, Canada

    template<class OffshoreSystemType, class DirectionalModel>
    void FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::GetFrequencyBandwidth(double &wmin, double &wmax) const {
      double m0 = std::pow(m_significant_height, 2) / 16.;
      double threshold = m0 * 0.01;

      wmin = dichotomySearch(1E-4, GetPeakFreq(mathutils::RADS), threshold);
      wmax = dichotomySearch(GetPeakFreq(mathutils::RADS), 10., threshold);
    }

    template<class OffshoreSystemType, class DirectionalModel>
    double FrWaveSpectrum<OffshoreSystemType, DirectionalModel>::dichotomySearch(double wmin, double wmax,
                                                                                 double threshold) const {

      double wresult, epsilon = 1.E-10;

      while (fabs(wmin - wmax) > epsilon * epsilon) {
        wresult = (wmin + wmax) / 2.0f;
        if (fabs(Eval(wresult) - threshold) < epsilon) {
          return wresult;
        } else {
          if ((Eval(wmax) - threshold) * (Eval(wresult) - threshold) < 0.0f)
            wmin = wresult;
          else
            wmax = wresult;
        }
      }
    }

    // =================================================================================================================
    // FrJonswapWaveSpectrum descriptions

    template<class OffshoreSystemType, class DirectionalModel>
    FrJonswapWaveSpectrum<OffshoreSystemType, DirectionalModel>::FrJonswapWaveSpectrum(double hs, double tp,
                                                                                       double gamma) :
        FrWaveSpectrum<OffshoreSystemType, DirectionalModel>(hs, tp),
        m_gamma(gamma) {
      CheckGamma();
    }

    template<class OffshoreSystemType, class DirectionalModel>
    void FrJonswapWaveSpectrum<OffshoreSystemType, DirectionalModel>::CheckGamma() {
      if (m_gamma < 1. || m_gamma > 10.) {
        // TODO: utiliser un vrai warning sur stderr
        std::cout << "WARNING: Valid values of gamma parameter in JONSWAP wave spectrum are between 1 and 10. "
                  << std::endl;
      }
    }

    template<class OffshoreSystemType, class DirectionalModel>
    double FrJonswapWaveSpectrum<OffshoreSystemType, DirectionalModel>::GetGamma() const { return m_gamma; }

    template<class OffshoreSystemType, class DirectionalModel>
    void FrJonswapWaveSpectrum<OffshoreSystemType, DirectionalModel>::SetGamma(double gamma) {
      m_gamma = gamma;
      CheckGamma();
    }

    template<class OffshoreSystemType, class DirectionalModel>
    double FrJonswapWaveSpectrum<OffshoreSystemType, DirectionalModel>::Eval(double w) const {

      double wp2 = this->m_peak_frequency * this->m_peak_frequency;
      double wp4 = wp2 * wp2;

      double w4_1 = std::pow(w, -4.);
      double w5_1 = w4_1 / w;

      double hs2 = this->m_significant_height * this->m_significant_height;

      double S_w = 0.;
      if (w > 0.) {

        S_w = 0.3125 * hs2 * wp4 * w5_1 * std::exp(-1.25 * wp4 * w4_1) * (1 - 0.287 * std::log(m_gamma));

        double a = std::exp(-std::pow(w - this->m_peak_frequency, 2) / (2. * wp2));
        if (w <= this->m_peak_frequency) {
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
    template<class OffshoreSystemType, class DirectionalModel>
    FrPiersonMoskowitzWaveSpectrum<OffshoreSystemType, DirectionalModel>::FrPiersonMoskowitzWaveSpectrum(double hs,
                                                                                                         double tp) :
        FrWaveSpectrum<OffshoreSystemType, DirectionalModel>(hs, tp) {}

    template<class OffshoreSystemType, class DirectionalModel>
    double FrPiersonMoskowitzWaveSpectrum<OffshoreSystemType, DirectionalModel>::Eval(double w) const {

      double Tz = mathutils::RADS2S(this->m_peak_frequency) / 1.408;  // Up-crossing period

      double A = std::pow(MU_2PI / Tz, 4) / (M_PI * std::pow(w, 4));

      double Hs2 = this->m_significant_height * this->m_significant_height;

      return 0.25 * A * (Hs2 / w) * std::exp(-A);

    }

    template<class OffshoreSystemType, class DirectionalModel>
    double FrTestWaveSpectrum<OffshoreSystemType, DirectionalModel>::Eval(double w) const {
      return 1.;
    }

}  // end namespace frydom
