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


//#include "MathUtils/Vector6d.h"

#include "FrLinearDamping.h"
//#include "frydom/core/common/FrException.h"
//
//#include "frydom/environment/FrEnvironmentInc.h"


namespace frydom {

    FrLinearDamping_::FrLinearDamping_(FLUID_TYPE ft, bool relativeToFluid) : m_fluidType(ft), m_relativeToFluid(relativeToFluid) {
        SetNull();
    }

    void FrLinearDamping_::SetNull() {
        m_dampingMatrix.SetNull();
    }

    void FrLinearDamping_::SetDampingMatrix(const FrLinearDamping_::DampingMatrix &dampingMatrix) {
        m_dampingMatrix = dampingMatrix;
    }

    void FrLinearDamping_::SetDiagonalDamping(double Du, double Dv, double Dw, double Dp, double Dq, double Dr) {
        SetDiagonalTranslationDamping(Du, Dv, Dw);
        SetDiagonalRotationDamping(Dp, Dq, Dr);
    }

    void FrLinearDamping_::SetDiagonalTranslationDamping(double Du, double Dv, double Dw) {
        m_dampingMatrix(0,0) = Du;
        m_dampingMatrix(1,1) = Dv;
        m_dampingMatrix(2,2) = Dw;
    }

    void FrLinearDamping_::SetDiagonalRotationDamping(double Dp, double Dq, double Dr) {
        m_dampingMatrix(3,3) = Dp;
        m_dampingMatrix(4,4) = Dq;
        m_dampingMatrix(5,5) = Dr;
    }

    void FrLinearDamping_::SetDampingCoeff(unsigned int iRow, unsigned int iCol, double coeff) {
        m_dampingMatrix(iRow, iCol) = coeff;
    }

    void FrLinearDamping_::SetRelativeToFluid(bool isRelative) {
        m_relativeToFluid = isRelative;
    }

    bool FrLinearDamping_::GetRelativeToFluid() {return m_relativeToFluid;}

    void FrLinearDamping_::Update(double time) {

        // Body Velocity at COG in body coordinates
        Velocity cogRelVel;
        if (m_relativeToFluid) {
            FrFrame_ cogFrame = m_body->GetFrameAtCOG(NWU);
            cogRelVel = -m_body->GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
                    cogFrame, m_body->GetCOGVelocityInWorld(NWU), m_fluidType, NWU);

        } else {
            cogRelVel = m_body->GetCOGVelocityInBody(NWU);
        }

        AngularVelocity rotVel = m_body->GetAngularVelocityInBody(NWU);

        GeneralizedVelocity genRelVel(cogRelVel, rotVel);

        GeneralizedForce genForce = - m_dampingMatrix * genRelVel;

        SetForceTorqueInBodyAtCOG(genForce.GetForce(), genForce.GetTorque(), NWU);

    }

    void FrLinearDamping_::Initialize() {
        FrForce_::Initialize();
        Check();
    }

    void FrLinearDamping_::StepFinalize() {
        FrForce_::StepFinalize();
    }

    void FrLinearDamping_::Check() const {
        // Here we check if every damping coefficient is positive
        for (unsigned int iRow=0; iRow<6; iRow++) {
            for (unsigned int iCol=0; iCol<6; iCol++) {
                if (m_dampingMatrix(iRow, iCol) < 0.) {
                    throw FrException("Damping coefficients cannot be negative !");
                }
            }
        }
    }

}  // end namespace frydom
