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

    FrQuadraticDamping_::FrQuadraticDamping_(FLUID_TYPE ft, bool relativeToFluid) :
            m_fluidType(ft), m_relative2Fluid(relativeToFluid) {}

    void FrQuadraticDamping_::SetDampingCoefficients(double Cu, double Cv, double Cw) {
        m_Cu = Cu;
        m_Cv = Cv;
        m_Cw = Cw;
    }

    void FrQuadraticDamping_::GetDampingCoefficients(double &Cu, double &Cv, double &Cw) {
        Cu = m_Cu;
        Cv = m_Cv;
        Cw = m_Cw;
    }

    void FrQuadraticDamping_::SetProjectedSections(double Su, double Sv, double Sw) {
        m_Su = Su;
        m_Sv = Sv;
        m_Sw = Sw;
    }

    void FrQuadraticDamping_::GetProjectedSections(double &Su, double &Sv, double &Sw) {
        Su = m_Su;
        Sv = m_Sv;
        Sw = m_Sw;
    }

    void FrQuadraticDamping_::SetRelative2Fluid(bool relativeVelocity) { m_relative2Fluid = relativeVelocity; }

    bool FrQuadraticDamping_::GetRelative2Fluid() {return m_relative2Fluid;}

    void FrQuadraticDamping_::Initialize() {
        FrForce_::Initialize();
    }

    void FrQuadraticDamping_::Update(double time) {

        Velocity cogRelVel;
        if (m_relative2Fluid) {
            FrFrame_ cogFrame = m_body->GetFrameAtCOG(NWU);
            cogRelVel = -m_body->GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
                    cogFrame, m_body->GetCOGVelocityInWorld(NWU), m_fluidType, NWU);
        } else {
            cogRelVel = m_body->GetCOGVelocityInBody(NWU);
        }

        double rho = m_body->GetSystem()->GetEnvironment()->GetFluidDensity(m_fluidType);

        double u = cogRelVel.GetVx();
        double v = cogRelVel.GetVy();
        double w = cogRelVel.GetVz();

        SetForceInBody(Force(
                - 0.5 * rho * m_Su * m_Cu * u*std::fabs(u),
                - 0.5 * rho * m_Sv * m_Cv * v*std::fabs(v),
                - 0.5 * rho * m_Sw * m_Cw * w*std::fabs(w)
                ), NWU);

    }

    void FrQuadraticDamping_::StepFinalize() {
        FrForce_::StepFinalize();
    }

}  // end namespace frydom
