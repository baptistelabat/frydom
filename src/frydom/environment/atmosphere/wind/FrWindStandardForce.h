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


#ifndef FRYDOM_FRWINDSTANDARDFORCE_H
#define FRYDOM_FRWINDSTANDARDFORCE_H

#include "frydom/core/force/FrForce.h"

namespace frydom {


  /// Standard wind drag force from DNV standard
  /// DNV-GL station keeping 01111

  /**
   * \class FrWindStandardForce
   * \brief Class for computing the wind loads following the DNV guidelines.
   */
  class FrWindStandardForce : public FrForce {

   private:
    double m_transverseArea = -1;
    double m_lateralArea = -1;
    double m_lpp = -1;
    double m_xCenter;

   public:

    explicit FrWindStandardForce(const std::string &name, FrBody *body);

    void SetLateralArea(double lateralArea);

    double GetLateralArea() const { return m_lateralArea; }

    void SetTransverseArea(double transverseArea);

    double GetTransverseArea() const { return m_transverseArea; }

    void SetXCenter(double xCenter);

    double GetXCenter() const { return m_xCenter; }

    void SetLenghtBetweenPerpendicular(double lpp);

    double GetLengthBetweenPerpendicular() const { return m_lpp; }

    void Initialize() override;

   private:

    /// Compute the wind force according to the DNV standards
    /// \param time Current time of the simulation from beginning, in seconds
    void Compute(double time) override;
  };


  std::shared_ptr<FrWindStandardForce>
  make_wind_standard_force(const std::string &name, std::shared_ptr<FrBody> body);


}  // end namespace frydom


#endif //FRYDOM_FRWINDSTANDARDFORCE_H
