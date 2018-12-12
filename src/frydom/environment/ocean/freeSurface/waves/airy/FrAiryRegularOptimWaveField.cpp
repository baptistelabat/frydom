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

    Complex frydom::FrAiryRegularOptimWaveField::GetComplexElevation(double x, double y) const {
        double kdir = x * c_cosTheta + y * c_sinTheta;
        return c_expJwt * exp(JJ * m_k * kdir);
    }

    Vector3d<frydom::Complex>
    FrAiryRegularOptimWaveField::GetComplexVelocity(double x, double y, double z) const {
        auto Vtemp = m_omega * GetComplexElevation(x, y) * m_verticalFactor->Eval(x, y, z, m_k, c_depth);

        auto Vx = c_cosTheta * Vtemp;
        auto Vy = c_sinTheta * Vtemp;
        auto Vz = -JJ * m_omega / m_k * GetComplexElevation(x, y) * m_verticalFactor->EvalDZ(x, y, z, m_k, c_depth);

        return {Vx, Vy, Vz};
    }

    FrAiryRegularOptimWaveField::FrAiryRegularOptimWaveField(frydom::FrFreeSurface_ *freeSurface)
            : FrAiryRegularWaveField(
            freeSurface) {

    }
}
