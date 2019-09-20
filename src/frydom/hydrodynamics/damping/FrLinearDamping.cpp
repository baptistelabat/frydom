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

    template<typename OffshoreSystemType>
    FrLinearDamping<OffshoreSystemType>::FrLinearDamping(FLUID_TYPE ft, bool relativeToFluid) : m_fluidType(ft),
                                                                            m_relativeToFluid(relativeToFluid) {
      SetNull();
    }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::SetNull() {
      m_dampingMatrix.SetNull();
    }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::SetDampingMatrix(const FrLinearDamping<OffshoreSystemType>::DampingMatrix &dampingMatrix) {
      m_dampingMatrix = dampingMatrix;
    }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::SetDiagonalDamping(double Du, double Dv, double Dw, double Dp, double Dq, double Dr) {
      SetDiagonalTranslationDamping(Du, Dv, Dw);
      SetDiagonalRotationDamping(Dp, Dq, Dr);
    }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::SetDiagonalTranslationDamping(double Du, double Dv, double Dw) {
      m_dampingMatrix(0, 0) = Du;
      m_dampingMatrix(1, 1) = Dv;
      m_dampingMatrix(2, 2) = Dw;
    }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::SetDiagonalRotationDamping(double Dp, double Dq, double Dr) {
      m_dampingMatrix(3, 3) = Dp;
      m_dampingMatrix(4, 4) = Dq;
      m_dampingMatrix(5, 5) = Dr;
    }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::SetDampingCoeff(unsigned int iRow, unsigned int iCol, double coeff) {
      m_dampingMatrix(iRow, iCol) = coeff;
    }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::SetRelativeToFluid(bool isRelative) {
      m_relativeToFluid = isRelative;
    }

    template<typename OffshoreSystemType>
    bool FrLinearDamping<OffshoreSystemType>::GetRelativeToFluid() { return m_relativeToFluid; }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::Compute(double time) {

      // Body Velocity at COG in body coordinates
      Velocity cogRelVel;
      if (m_relativeToFluid) {
        FrFrame cogFrame = this->m_body->GetFrameAtCOG(NWU);
        cogRelVel = -this->m_body->GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
            cogFrame, this->m_body->GetCOGLinearVelocityInWorld(NWU), m_fluidType, NWU);

      } else {
        cogRelVel = this->m_body->GetCOGVelocityInBody(NWU);
      }

      AngularVelocity rotVel = this->m_body->GetAngularVelocityInBody(NWU);

      GeneralizedVelocity genRelVel(cogRelVel, rotVel);

      GeneralizedForce genForce = -m_dampingMatrix * genRelVel;

      this->SetForceTorqueInBodyAtCOG(genForce.GetForce(), genForce.GetTorque(), NWU);

    }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::Initialize() {
      FrForce<OffshoreSystemType>::Initialize();
      Check();
    }

    template<typename OffshoreSystemType>
    void FrLinearDamping<OffshoreSystemType>::Check() const {
      // Here we check if every damping coefficient is positive
      for (unsigned int iRow = 0; iRow < 6; iRow++) {
        for (unsigned int iCol = 0; iCol < 6; iCol++) {
          if (m_dampingMatrix(iRow, iCol) < 0.) {
            throw FrException("Damping coefficients cannot be negative !");
          }
        }
      }
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearDamping<OffshoreSystemType>>
    make_linear_damping_force(std::shared_ptr<FrBody<OffshoreSystemType>> body, FLUID_TYPE ft, bool relativeToFluid) {

      // This function creates a linear damping force.

      // Construction of the linear damping force object.
      auto forceLinearDamping = std::make_shared<FrLinearDamping<OffshoreSystemType>>(ft, relativeToFluid);

      // Add the linear damping force object as an external force to the body.
      body->AddExternalForce(forceLinearDamping);

      return forceLinearDamping;
    }

}  // end namespace frydom
