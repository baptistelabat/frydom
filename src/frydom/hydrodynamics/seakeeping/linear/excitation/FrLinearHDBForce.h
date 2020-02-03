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


#ifndef FRYDOM_FRLINEARHDBFORCE_H
#define FRYDOM_FRLINEARHDBFORCE_H

#include "frydom/core/math/FrVector.h"
#include "MathUtils/Interp1d.h"
#include "frydom/core/force/FrForce.h"

namespace frydom {

  // Forward declarations.
  class FrHydroDB;

  /**
   * \class FrLinearHDBForce
   * \brief Virtual class for defining a hydrodynamic linear model force :
   * see FrLinearDiffractionForce, FrLinearExcitationForce, FrLinearFroudeKrylovForce, for derived implementations.
   */
  class FrLinearHDBForce : public FrForce {

   protected:

    /// Interpolator in waves frequencies and directions.
    std::vector<std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators;

    /// Hydrodynamic database.
    std::shared_ptr<FrHydroDB> m_HDB;

    /// Hydrodynamic components.
    std::vector<Eigen::MatrixXcd> m_Fhdb;


   public:

    /// Constructor.
    FrLinearHDBForce(const std::string &name,
                     const std::string &type_name,
                     FrBody *body,
                     const std::shared_ptr<FrHydroDB> &HDB);

    virtual Eigen::MatrixXcd GetHDBData(unsigned int iangle) const = 0;

    virtual Eigen::VectorXcd GetHDBData(unsigned int iangle, unsigned iforce) const = 0;

    /// Build the interpolators between hydrodynamic components and wave frequency and direction discretizations
    virtual void BuildHDBInterpolators();

    /// This function return the excitation force (linear excitation) or the diffraction force (nonlinear excitation) form the interpolator.
    /// Interpolates the hydrodynamic components with respect to the wave frequency and directions discretizations given
    /// \param waveFrequencies wave frequency discretization
    /// \param waveDirections wave direction discretization
    /// \return interpolation of the hydrodynamic component
    std::vector<Eigen::MatrixXcd> GetHDBInterp(std::vector<double> waveFrequencies,
                                               std::vector<double> waveDirections);

    void Initialize() override;

   protected:

    void Compute(double time) override;

    /// This function computes the excitation loads (linear excitation) or the diffraction loads (nonlinear excitation).
    void Compute_F_HDB();

  };


} // end namespace frydom

#endif //FRYDOM_FRLINEARHDBFORCE_H
