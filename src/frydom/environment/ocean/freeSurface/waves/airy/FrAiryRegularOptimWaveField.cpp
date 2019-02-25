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


#include "FrAiryRegularOptimWaveField.h"

namespace frydom {

    void FrAiryRegularOptimWaveField::InternalUpdate() {
        c_expJwt = m_height * exp(-JJ * m_omega * c_time); // m_height = Amplitude.
        c_cosTheta = cos(m_dirAngle);
        c_sinTheta = sin(m_dirAngle);
    }

    void FrAiryRegularOptimWaveField::Initialize() {
        FrWaveField_::Initialize();
        InternalUpdate();
    }

    void FrAiryRegularOptimWaveField::StepFinalize() {
        FrWaveField_::StepFinalize();
        InternalUpdate();
    }

    std::vector<std::vector<Complex>> frydom::FrAiryRegularOptimWaveField::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {
        double NWUsign = 1;
        if(IsNED(fc)) {y=-y; NWUsign = -NWUsign;}
        double kdir = x * c_cosTheta + y * c_sinTheta;
        Complex cmplxElevation = c_expJwt * exp(JJ * m_k * kdir) * NWUsign * c_ramp;
        return std::vector<std::vector<Complex>>(1, std::vector<Complex>(1, cmplxElevation));
    }

    Vector3d<frydom::Complex>
    FrAiryRegularOptimWaveField::GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
        double NWUsign = 1;
        if(IsNED(fc)) {y=-y; z=-z; NWUsign = -NWUsign;}
        auto ComplexElevation = GetComplexElevation(x, y, fc);

        auto Vtemp = m_omega * ComplexElevation[0][0] * m_verticalFactor->Eval(x, y, z, m_k, c_depth);

        auto Vx = c_cosTheta * Vtemp * NWUsign;
        auto Vy = c_sinTheta * Vtemp;
        auto Vz = -JJ * m_omega / m_k * ComplexElevation[0][0] * m_verticalFactor->EvalDZ(x, y, z, m_k, c_depth);

        return {Vx, Vy, Vz};
    }

    FrAiryRegularOptimWaveField::FrAiryRegularOptimWaveField(frydom::FrFreeSurface_ *freeSurface)
            : FrAiryRegularWaveField(
            freeSurface) {

    }

}  // end namespace frydom
