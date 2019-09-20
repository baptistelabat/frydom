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

#include "FrDirectionalModel.h"
#include <cmath>
#include <iostream>

namespace frydom {

    // FrWaveDirectionalModel descriptions

    std::vector<double>
    FrWaveDirectionalModel::GetSpreadingFunction(const std::vector<double> &thetaVect, double theta_mean) {
      std::vector<double> spreading_fcn;
      spreading_fcn.reserve(thetaVect.size());
      double spreadEval;

      for (double theta: thetaVect) {
        spreadEval = Eval(theta, theta_mean);
        spreading_fcn.push_back(spreadEval);
      }
      return spreading_fcn;
    }

    void FrWaveDirectionalModel::GetDirectionBandwidth(double &theta_min, double &theta_max, double theta_mean) const {
      double threshold = 0.01;

      theta_min = dichotomySearch(theta_mean, threshold);
      double Dtheta = theta_mean - theta_min;
      theta_max = theta_min + 2. * Dtheta;
    }

    double FrWaveDirectionalModel::dichotomySearch(double theta_mean, double threshold) const {
      double theta_min = theta_mean - M_PI;
      double theta_max = theta_mean;

      double theta_result, epsilon = 1.E-10;

      while (fabs(theta_min - theta_max) > epsilon * epsilon) {
        theta_result = (theta_min + theta_max) / 2.0f;
        if (fabs(Eval(theta_result, theta_mean) - threshold) < epsilon) {
          return theta_result;
        } else {
          if ((Eval(theta_max, theta_mean) - threshold) * (Eval(theta_result, theta_mean) - threshold) < 0.0f)
            theta_min = theta_result;
          else
            theta_max = theta_result;
        }
      }
      // FIXME : rien n'est renvoye !!!
    }


    // FrMonoDirectionalModel

    double FrMonoDirectionalModel::Eval(double theta, double theta_mean) const {
      return (theta == theta_mean) ? 1 : 0;
    }



    // FrCos2sDirectionalModel descriptions

    void FrCos2sDirectionalModel::CheckSpreadingFactor() {
      // TODO: utiliser un warning ver la stderr
      if (m_spreading_factor < 1. || m_spreading_factor > 100.) {
        std::cout << "The spreading factor of a cos2s directional spectrum model should lie between 1. and 100."
                  << std::endl;
      }
    }

    FrCos2sDirectionalModel::FrCos2sDirectionalModel(double spreading_factor) : m_spreading_factor(spreading_factor) {
      CheckSpreadingFactor();
      EvalCs();
    }

//    WAVE_DIRECTIONAL_MODEL FrCos2sDirectionalModel::GetType() const { return COS2S; }

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
      c_s = (pow(2., two_s - 1.) / M_PI) * pow(std::tgamma(s + 1.), 2.) / std::tgamma(two_s + 1.);
    }

    // =================================================================================================================
    // FrTestDirectionalModel descriptions

//    WAVE_DIRECTIONAL_MODEL FrTestDirectionalModel::GetType() const { return DIRTEST; }

    double FrTestDirectionalModel::Eval(double theta, double theta_mean) const {
      return 1.;
    }

}  // end namespace frydom
