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


#include "FrLinearDamping.h"

#include "frydom/core/common/FrFrame.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"


namespace frydom {

  FrLinearDamping::FrLinearDamping(const std::string &name,
                                   FrBody *body,
                                   FLUID_TYPE ft,
                                   bool relativeToFluid) :

      FrForce(name, body),
      m_fluidType(ft),
      m_relativeToFluid(relativeToFluid) {
    SetNull();
  }

  void FrLinearDamping::SetNull() {
    m_dampingMatrix.SetNull();
  }

  void FrLinearDamping::SetDampingMatrix(const FrLinearDamping::DampingMatrix &dampingMatrix) {
    m_dampingMatrix = dampingMatrix;
  }

  void FrLinearDamping::SetDiagonalDamping(double Du, double Dv, double Dw, double Dp, double Dq, double Dr) {
    SetDiagonalTranslationDamping(Du, Dv, Dw);
    SetDiagonalRotationDamping(Dp, Dq, Dr);
  }

  void FrLinearDamping::SetDiagonalTranslationDamping(double Du, double Dv, double Dw) {
    m_dampingMatrix(0, 0) = Du;
    m_dampingMatrix(1, 1) = Dv;
    m_dampingMatrix(2, 2) = Dw;
  }

  void FrLinearDamping::SetDiagonalRotationDamping(double Dp, double Dq, double Dr) {
    m_dampingMatrix(3, 3) = Dp;
    m_dampingMatrix(4, 4) = Dq;
    m_dampingMatrix(5, 5) = Dr;
  }

  void FrLinearDamping::SetDampingCoeff(unsigned int iRow, unsigned int iCol, double coeff) {
    m_dampingMatrix(iRow, iCol) = coeff;
  }

  void FrLinearDamping::SetRelativeToFluid(bool isRelative) {
    m_relativeToFluid = isRelative;
  }

  bool FrLinearDamping::GetRelativeToFluid() { return m_relativeToFluid; }

  void FrLinearDamping::Compute(double time) {

    auto body = GetBody();

    // Body Velocity at COG in body coordinates
    Velocity cogRelVel;
    if (m_relativeToFluid) {
      FrFrame cogFrame = body->GetFrameAtCOG(NWU);
      cogRelVel = -body->GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
          cogFrame, body->GetCOGLinearVelocityInWorld(NWU), m_fluidType, NWU);

    } else {
      cogRelVel = body->GetCOGVelocityInBody(NWU);
    }

    AngularVelocity rotVel = body->GetAngularVelocityInBody(NWU);

    GeneralizedVelocity genRelVel(cogRelVel, rotVel);

    GeneralizedForce genForce = -m_dampingMatrix * genRelVel;

    SetForceTorqueInBodyAtCOG(genForce.GetForce(), genForce.GetTorque(), NWU);

  }

  void FrLinearDamping::Initialize() {
    FrForce::Initialize();
    Check();
  }

  void FrLinearDamping::Check() const {
    // Here we check if every damping coefficient is positive
    for (unsigned int iRow = 0; iRow < 6; iRow++) {
      for (unsigned int iCol = 0; iCol < 6; iCol++) {
        if (m_dampingMatrix(iRow, iCol) < 0.) { // FIXME : utiliser eigen pour le check !!!
          throw FrException("Damping coefficients cannot be negative !");
        }
      }
    }
  }

  std::shared_ptr<FrLinearDamping>
  make_linear_damping_force(const std::string &name,
                            std::shared_ptr<FrBody> body,
                            FLUID_TYPE ft,
                            bool relativeToFluid) {

    // This function creates a linear damping force.

    // Construction of the linear damping force object.
    auto forceLinearDamping = std::make_shared<FrLinearDamping>(name, body.get(), ft, relativeToFluid);

    // Add the linear damping force object as an external force to the body.
    body->AddExternalForce(forceLinearDamping);

    return forceLinearDamping;
  }

}  // end namespace frydom
