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

    FrQuadraticDamping::FrQuadraticDamping(const std::string& name, FLUID_TYPE ft, bool relativeToFluid) :
        FrForce(name), m_fluidType(ft), m_relative2Fluid(relativeToFluid) {}

    void FrQuadraticDamping::SetDampingCoefficients(double Cu, double Cv, double Cw) {
      m_Cu = Cu;
      m_Cv = Cv;
      m_Cw = Cw;
    }

    void FrQuadraticDamping::GetDampingCoefficients(double &Cu, double &Cv, double &Cw) {
      Cu = m_Cu;
      Cv = m_Cv;
      Cw = m_Cw;
    }

    void FrQuadraticDamping::SetProjectedSections(double Su, double Sv, double Sw) {
      m_Su = Su;
      m_Sv = Sv;
      m_Sw = Sw;
    }

    void FrQuadraticDamping::GetProjectedSections(double &Su, double &Sv, double &Sw) {
      Su = m_Su;
      Sv = m_Sv;
      Sw = m_Sw;
    }

    void FrQuadraticDamping::SetRelative2Fluid(bool relativeVelocity) { m_relative2Fluid = relativeVelocity; }

    bool FrQuadraticDamping::GetRelative2Fluid() { return m_relative2Fluid; }

    void FrQuadraticDamping::Compute(double time) {

      auto body = GetBody();

      Velocity cogRelVel;
      if (m_relative2Fluid) {
        FrFrame cogFrame = body->GetFrameAtCOG(NWU);
        cogRelVel = -body->GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
            cogFrame, body->GetCOGLinearVelocityInWorld(NWU), m_fluidType, NWU);
      } else {
        cogRelVel = body->GetCOGVelocityInBody(NWU);
      }

      double rho = body->GetSystem()->GetEnvironment()->GetFluidDensity(m_fluidType);

      double u = cogRelVel.GetVx();
      double v = cogRelVel.GetVy();
      double w = cogRelVel.GetVz();

      SetForceInBody(Force(
          -0.5 * rho * m_Su * m_Cu * u * std::fabs(u),
          -0.5 * rho * m_Sv * m_Cv * v * std::fabs(v),
          -0.5 * rho * m_Sw * m_Cw * w * std::fabs(w)
      ), NWU);

    }

    std::shared_ptr<FrQuadraticDamping>
    make_quadratic_damping_force(const std::string& name,
                                 std::shared_ptr<FrBody> body,
                                 FLUID_TYPE ft,
                                 bool relativeToFluid) {

      // This function creates a linear damping force.

      // Construction of the linear damping force object.
      auto forceQuadraticDamping = std::make_shared<FrQuadraticDamping>(name, ft, relativeToFluid);

      // Add the linear damping force object as an external force to the body.
      body->AddExternalForce(forceQuadraticDamping);

      return forceQuadraticDamping;
    }

}  // end namespace frydom
