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


#include "FrQuadraticDamping.h"

#include "frydom/core/common/FrFrame.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrQuadraticDamping<OffshoreSystemType>::FrQuadraticDamping(FLUID_TYPE ft, bool relativeToFluid) :
        m_fluidType(ft), m_relative2Fluid(relativeToFluid) {}

    template<typename OffshoreSystemType>
    void FrQuadraticDamping<OffshoreSystemType>::SetDampingCoefficients(double Cu, double Cv, double Cw) {
      m_Cu = Cu;
      m_Cv = Cv;
      m_Cw = Cw;
    }

    template<typename OffshoreSystemType>
    void FrQuadraticDamping<OffshoreSystemType>::GetDampingCoefficients(double &Cu, double &Cv, double &Cw) {
      Cu = m_Cu;
      Cv = m_Cv;
      Cw = m_Cw;
    }

    template<typename OffshoreSystemType>
    void FrQuadraticDamping<OffshoreSystemType>::SetProjectedSections(double Su, double Sv, double Sw) {
      m_Su = Su;
      m_Sv = Sv;
      m_Sw = Sw;
    }

    template<typename OffshoreSystemType>
    void FrQuadraticDamping<OffshoreSystemType>::GetProjectedSections(double &Su, double &Sv, double &Sw) {
      Su = m_Su;
      Sv = m_Sv;
      Sw = m_Sw;
    }

    template<typename OffshoreSystemType>
    void FrQuadraticDamping<OffshoreSystemType>::SetRelative2Fluid(bool relativeVelocity) { m_relative2Fluid = relativeVelocity; }

    template<typename OffshoreSystemType>
    bool FrQuadraticDamping<OffshoreSystemType>::GetRelative2Fluid() { return m_relative2Fluid; }

    template<typename OffshoreSystemType>
    void FrQuadraticDamping<OffshoreSystemType>::Compute(double time) {

      Velocity cogRelVel;
      if (m_relative2Fluid) {
        FrFrame cogFrame = this->m_body->GetFrameAtCOG(NWU);
        cogRelVel = -this->m_body->GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
            cogFrame, this->m_body->GetCOGLinearVelocityInWorld(NWU), m_fluidType, NWU);
      } else {
        cogRelVel = this->m_body->GetCOGVelocityInBody(NWU);
      }

      double rho = this->m_body->GetSystem()->GetEnvironment()->GetFluidDensity(m_fluidType);

      double u = cogRelVel.GetVx();
      double v = cogRelVel.GetVy();
      double w = cogRelVel.GetVz();

      this->SetForceInBody(Force(
          -0.5 * rho * m_Su * m_Cu * u * std::fabs(u),
          -0.5 * rho * m_Sv * m_Cv * v * std::fabs(v),
          -0.5 * rho * m_Sw * m_Cw * w * std::fabs(w)
      ), NWU);

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrQuadraticDamping<OffshoreSystemType>>
    make_quadratic_damping_force(std::shared_ptr<FrBody<OffshoreSystemType>> body, FLUID_TYPE ft, bool relativeToFluid) {

      // This function creates a linear damping force.

      // Construction of the linear damping force object.
      auto forceQuadraticDamping = std::make_shared<FrQuadraticDamping<OffshoreSystemType>>(ft, relativeToFluid);

      // Add the linear damping force object as an external force to the body.
      body->AddExternalForce(forceQuadraticDamping);

      return forceQuadraticDamping;
    }

}  // end namespace frydom
