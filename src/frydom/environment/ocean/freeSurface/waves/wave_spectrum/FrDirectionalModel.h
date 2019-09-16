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

#ifndef FRYDOM_FRDIRECTIONALMODEL_H
#define FRYDOM_FRDIRECTIONALMODEL_H

#include <vector>


namespace frydom {

//    enum WAVE_DIRECTIONAL_MODEL {  // TODO : passer dans la classe...
//        NONE,
//        COS2S,
//        DIRTEST
//    };

    //TODO: Changer la discrétisation en direction, de manière à obtenir une énergie spectrale constante pour toutes les composantes
    // voir OrcaFlex Equal Energy dans Frequency spectrum discretisation.
    /**
     * \class FrWaveDirectionalModel
     * \brief Class for setting the wave directional model.
     */
    class FrWaveDirectionalModel {
     public:

//        /// Get the type of the directional model
//        /// \return type of directional model (COS2S/NONE/DIRTEST)
//        virtual WAVE_DIRECTIONAL_MODEL GetType() const  = 0;

      /// Get the spreading function for a vector of wave directions thetaVect
      /// \param thetaVect vector of wave directions
      /// \param theta_mean mean wave direction
      /// \return spreading function
      virtual std::vector<double> GetSpreadingFunction(const std::vector<double> &thetaVect, double theta_mean);

      /// Evaluate the spreading function for a wave direction
      /// \param theta wave direction
      /// \param theta_mean mean wave direction
      /// \return evaluation of the spreading function
      virtual double Eval(double theta, double theta_mean) const = 0;

      /// Get the extremal frequency values where the power spectral density is equal to 1% of the total variance m0 = hs**2/16
      /// \return extremal frequency values
      void GetDirectionBandwidth(double &theta_min, double &theta_max, double theta_mean) const;
      // FIXME  : verifier que cette methode fonctionne egalement pour le monodirectionnel

     protected:

      /// Search by dichotomy method.
      /// \param theta_mean mean wave direction
      /// \param threshold threshold
      /// \return
      double dichotomySearch(double theta_mean, double threshold) const;

    };


    class FrMonoDirectionalModel : public FrWaveDirectionalModel {

     private:
      double m_wave_direction;

     public:

      double Eval(double theta, double theta_mean) const override;

    };


    // =================================================================================================================
    /// -------------------------------------------------------------------
    /// FrCos2sDirectionalModel
    /// -------------------------------------------------------------------
    /// This directional model, proposed by Longuet-Higgins[1963] is an extension of the cosine-squared model.
    /// the spreading function is given by
    /// spreading_fcn = c_s * cos^2s[(theta-theta0)/2]
    /// where c_s = [ 2^(2s-1)]/Pi . [Gamma²(s+1)]/[Gamma(2s+1)]
    class FrCos2sDirectionalModel : public FrWaveDirectionalModel {

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
      explicit FrCos2sDirectionalModel(double spreading_factor = 10.);

//        /// Get the type of the directional model
//        /// \return type of directional model, here COS2S
//        WAVE_DIRECTIONAL_MODEL GetType() const override;

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
    class FrTestDirectionalModel : public FrWaveDirectionalModel {
     public:

//        /// Get the type of the directional model
//        /// \return type of directional model, here DIRTEST
//        WAVE_DIRECTIONAL_MODEL GetType() const override;

      /// Evaluate the spreading function for a wave direction
      /// \param theta wave direction
      /// \param theta_mean mean wave direction
      /// \return evaluation of the spreading function
      double Eval(double theta, double theta_mean) const override;
    };

    // =================================================================================================================


}  // end namespace frydom



#endif //FRYDOM_FRDIRECTIONALMODEL_H
