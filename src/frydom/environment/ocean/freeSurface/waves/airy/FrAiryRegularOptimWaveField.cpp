//
// Created by Lucas Letournel on 11/12/18.
//

#include "FrAiryRegularOptimWaveField.h"

namespace frydom {

    void FrAiryRegularOptimWaveField::InternalUpdate() {
        c_expJwt = m_height * exp(-JJ * m_omega * c_time);
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

    Complex frydom::FrAiryRegularOptimWaveField::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {
        double NWUsign = 1;
        if(IsNED(fc)) {y=-y; NWUsign = -NWUsign;}
        double kdir = x * c_cosTheta + y * c_sinTheta;
        return c_expJwt * exp(JJ * m_k * kdir) * NWUsign;
    }

    Vector3d<frydom::Complex>
    FrAiryRegularOptimWaveField::GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
        double NWUsign = 1;
        if(IsNED(fc)) {y=-y; z=-z; NWUsign = -NWUsign;}
        auto ComplexElevation = GetComplexElevation(x, y, fc);

        auto Vtemp = m_omega * ComplexElevation * m_verticalFactor->Eval(x, y, z, m_k, c_depth);

        auto Vx = c_cosTheta * Vtemp;
        auto Vy = c_sinTheta * Vtemp * NWUsign;
        auto Vz = -JJ * m_omega / m_k * ComplexElevation * m_verticalFactor->EvalDZ(x, y, z, m_k, c_depth) * NWUsign;

        return {Vx, Vy, Vz};
    }

    FrAiryRegularOptimWaveField::FrAiryRegularOptimWaveField(frydom::FrFreeSurface_ *freeSurface)
            : FrAiryRegularWaveField(
            freeSurface) {

    }
}
