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


#ifndef FRYDOM_FRLINEARDAMPING_H
#define FRYDOM_FRLINEARDAMPING_H

#include "MathUtils/Matrix66.h"

#include "frydom/core/force/FrForce.h"
#include "frydom/environment/FrFluidType.h"


namespace frydom {


  /**
   * \class FrLinearDamping
   * \brief Class for computing additional linear damping loads.
   */
  class FrLinearDamping : public FrForce {

   public:
    using DampingMatrix = mathutils::Matrix66<double>; // TODO : disposer d'une Matrix66 dans mathutils

   private:

    DampingMatrix m_dampingMatrix;

    /// WATER or AIR.
    FLUID_TYPE m_fluidType;

    /// Velocity to use: fluid felocity (true) or body velocity (false).
    bool m_relativeToFluid = false;  // FIXME : on doit pouvoir aussi appliquer dans l'air !!!!!

   public:

    FrLinearDamping(const std::string &name, FrBody *body, FLUID_TYPE ft, bool relativeToFluid);

    /// Return true if the force is included in the static analysis
    bool IncludedInStaticAnalysis() const override { return true; }

    void SetNull();

    /// Setter for the whole damping matrix. Translations are upper left and rotations are lower right.
    void SetDampingMatrix(const DampingMatrix &dampingMatrix);

    /// Setter for the diagonal components of the damping matrix
    void SetDiagonalDamping(double Du, double Dv, double Dw, double Dp, double Dq, double Dr);

    /// Setter for the diagonal components in translation of the damping matrix
    void SetDiagonalTranslationDamping(double Du, double Dv, double Dw);

    /// Setter for the diagonal components in rotation of the damping matrix
    void SetDiagonalRotationDamping(double Dp, double Dq, double Dr);

    /// Set a damping coefficient given its position.
    void SetDampingCoeff(unsigned int iRow, unsigned int iCol, double coeff);

    /// Setter for the boolean : m_relativeVelocity
    void SetRelativeToFluid(bool isRelative);

    /// Getter for the boolean : m_relativeVelocity
    bool GetRelativeToFluid();

    void Initialize() override;

   private:

    /// Compute the linear damping force
    /// \param time Current time of the simulation from beginning, in seconds
    void Compute(double time) override;

    void Check() const;

  };

  /// This function creates a linear damping force.
  std::shared_ptr<FrLinearDamping>
  make_linear_damping_force(const std::string &name,
                            std::shared_ptr<FrBody> body,
                            FLUID_TYPE ft,
                            bool relativeToFluid);

}  // end namespace frydom

#endif //FRYDOM_FRLINEARDAMPING_H
