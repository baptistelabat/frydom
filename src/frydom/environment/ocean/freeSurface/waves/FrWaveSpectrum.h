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


#ifndef FRYDOM_FRWAVESPECTRUM_H
#define FRYDOM_FRWAVESPECTRUM_H

#include <memory>
#include <vector>

#include "frydom/core/common/FrObject.h"
#include "frydom/core/common/FrUnits.h"

namespace frydom {

  enum WAVE_DIRECTIONAL_MODEL {  // TODO : passer dans la classe...
    NONE,
    COS2S,
    DIRTEST
  };

  //TODO: Changer la discrétisation en direction, de manière à obtenir une énergie spectrale constante pour toutes les composantes
  // voir OrcaFlex Equal Energy dans Frequency spectrum discretisation.
  /**
   * \class FrWaveDirectionalModel
   * \brief Class for setting the wave directional model.
   */
  class FrWaveDirectionalModel {
   public:

    /// Get the type of the directional model
    /// \return type of directional model (COS2S/NONE/DIRTEST)
    virtual WAVE_DIRECTIONAL_MODEL GetType() const = 0;

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

    /// Get the type of the directional model
    /// \return type of directional model, here COS2S
    WAVE_DIRECTIONAL_MODEL GetType() const override;

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

    /// Get the type of the directional model
    /// \return type of directional model, here DIRTEST
    WAVE_DIRECTIONAL_MODEL GetType() const override;

    /// Evaluate the spreading function for a wave direction
    /// \param theta wave direction
    /// \param theta_mean mean wave direction
    /// \return evaluation of the spreading function
    double Eval(double theta, double theta_mean) const override;
  };

  // =================================================================================================================
  /// -------------------------------------------------------------------
  /// FrWaveSpectrum
  /// -------------------------------------------------------------------
  /// Virtual base class for the wave spectra.

  //TODO: Changer la discrétisation en fréquence, de manière à obtenir une énergie spectrale constante pour toutes les composantes
  // voir OrcaFlex Equal Energy dans Frequency spectrum discretisation.
  class FrWaveSpectrum : public FrObject {

   protected:
    double m_significant_height = 3.;       ///< Significant height (in meters): mean wave height (trough to crest)
    ///<  of the highest third of the waves (H1/3)
    double m_peak_frequency = mathutils::S2RADS(9.);   ///< Peak circular frequency (in radians/s),
    ///< is the wave frequency with the highest energy

    WAVE_DIRECTIONAL_MODEL m_dir_model_type = NONE;   ///< wave directional model type (NONE/COS2S/DIRTEST)

    std::unique_ptr<FrWaveDirectionalModel> m_directional_model = nullptr; ///< wave directional model

   public:

    /// Default constructor of the wave spectrum
    FrWaveSpectrum() = default;

    /// Constructor of the wave spectrum, based on a significant height and peak period, with its associated unit.
    /// \param hs significant height
    /// \param tp peak period
    FrWaveSpectrum(double hs, double tp);

    /// Set the wave directional model to a cos2s, with a spreading factor
    /// \param spreadingFactor spreading factor of the cos2s model
    void SetCos2sDirectionalModel(double spreadingFactor);

    /// Set the wave directional model to use from type
    /// \param model wave directional model type
    void SetDirectionalModel(WAVE_DIRECTIONAL_MODEL model);

    /// Set the wave directional model to use from object
    /// \param dir_model wave directional model
    void SetDirectionalModel(FrWaveDirectionalModel *dir_model);

    /// Get the wave directional model
    /// \return wave directional model
    FrWaveDirectionalModel *GetDirectionalModel() const;

    /// Set the wave spectrum as multi-directional
    /// \param model wave directional model type
    void DirectionalON(WAVE_DIRECTIONAL_MODEL model = COS2S);

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
    /// \param Fp peak frequency
    /// \param unit unit of the peak frequency (S/RADS/HZ/...)
    void SetPeakFreq(double Fp, FREQUENCY_UNIT unit);

    /// Set the significant height and peak frequency
    /// \param Hs significant height
    /// \param Tp peak frequency
    /// \param unit unit of the peak frequency (S/RADS/HZ/...)
    void SetHsTp(double Hs, double Tp, FREQUENCY_UNIT unit);

    /// Get the extremal frequency values where the power spectral density is equal to 1% of the total variance m0 = hs**2/16
    /// \return extremal frequency values
    void GetFrequencyBandwidth(double &wmin, double &wmax) const;

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
    virtual std::vector<std::vector<double>>
    GetWaveAmplitudes(std::vector<double> waveFrequencies, std::vector<double> waveDirections);


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
                                                               unsigned int nb_dir, double theta_min, double theta_max,
                                                               double theta_mean);

    /// Initialize the state of the wave spectrum
    void Initialize() override {}

    /// Method called at the send of a time step. Logging may be used here
    void StepFinalize() override {}

   private:

    /// Search by dichotomy method.
    /// \param wmin minimium value
    /// \param wmax maximum value
    /// \param threshold threshold
    /// \return result
    double dichotomySearch(double wmin, double wmax, double threshold) const;

  };

  // =================================================================================================================
  // FIXME: est-ce vraiment critique que d'avoir les 2 choses suivantes en macro ???
  #define _SIGMA2_1_left (1./(0.07*0.07))
  #define _SIGMA2_1_right (1./(0.09*0.09))
  /// -------------------------------------------------------------------
  /// FrJonswapWaveSpectrum
  /// -------------------------------------------------------------------
  /// Class for a JONSWAP wave spectrum
  ///
  ///    References
  ///    ----------
  ///    DNV, Modelling and analysis of marine operations. Offshore Standard, 2011.
  ///    Kim C.H., Nonlinear Waves and Offshore structures, 2008
  ///    Molin B., Hydrodynamique des Structures Offshore, 2002
  ///
  //TODO : Implémenter les recommandations de DNV pour la valeur de gamma.
  //TODO : S'appuyer sur le calcul du spectre de Pierson-Moskowitz pour celui de JONSWAP?
  /**
   * \class FrJonswapWaveSpectrum
   * \brief Class for defining a Jonswap wave spectrum.
   */
  class FrJonswapWaveSpectrum : public FrWaveSpectrum {

   private:
    double m_gamma = 3.3;   ///< Peakedness factor of the Jonswap wave spectrum,
    ///< it is the ratio of the maximum of JONSWAP wave spectral density
    ///< to the maximum of Pierson-Moskowitz spectral density.

   public:

    /// Default constructor
    FrJonswapWaveSpectrum() = default;

    /// Constructor for a JONSWAP wave spectrum, based on the significant height, peak frequency and its associated unit
    /// and a gamma factor
    /// \param hs significant height
    /// \param tp peak frequency
    /// \param gamma gamma factor of the Jonswap spectrum
    FrJonswapWaveSpectrum(double hs, double tp,
                          double gamma); // TODO : virer le FREQUENCY_UNIT, on passe des secondes quoi qu'il arrive

    /// Check that the gamma factor is correctly defined between 1. and 10.
    void CheckGamma();

    /// Get the gamma factor of the JONSWAP spectrum
    /// \return gamma factor of the JONSWAP spectrum
    double GetGamma() const;

    /// Get the gamma factor of the JONSWAP spectrum
    /// \param gamma gamma factor of the JONSWAP spectrum
    void SetGamma(double gamma);

    /// Eval the spectrum at one frequency
    /// \param w circular frequency for which the wave spectrum is evaluated
    /// \return evaluation of the wave spectrum
    double Eval(double w) const final;

  };


  // =================================================================================================================
  /// -------------------------------------------------------------------
  /// FrPiersonMoskowitzWaveSpectrum
  /// -------------------------------------------------------------------
  /// Class for a Pierson Moskowitz wave spectrum
  /**
   * \class FrPiersonMoskowitzWaveSpectrum
   * \brief Class for defining a Pierson-Moskowitz wave spectrum.
   */
  class FrPiersonMoskowitzWaveSpectrum : public FrWaveSpectrum {

   public:

    /// Default constructor of a Pierson Moskowitz wave spectrum
    FrPiersonMoskowitzWaveSpectrum() = default;

    /// Constructor for a Pierson Moskowitz wave spectrum, based on the significant height, peak frequency
    /// and its associated unit
    /// \param hs significant height
    /// \param tp peak frequency
    FrPiersonMoskowitzWaveSpectrum(double hs, double tp);

    /// Eval the spectrum at one frequency
    /// \param w circular frequency for which the wave spectrum is evaluated
    /// \return evaluation of the wave spectrum
    double Eval(double w) const final;

  };


  // =================================================================================================================
  /// For test use only
  class FrTestWaveSpectrum : public FrWaveSpectrum {
   public:
    FrTestWaveSpectrum() = default;

    double Eval(double w) const final;

  };

}  // end namespace frydom


#endif //FRYDOM_FRWAVESPECTRUM_H
