//
// Created by frongere on 07/02/19.
//

#include "MathUtils/MathUtils.h"

#include "FrSinFunction.h"


namespace frydom {

    FrSinFunction::FrSinFunction() = default;

    void FrSinFunction::SetAmplitude(double amplitude) {
        m_amplitude = amplitude;
    }

    double FrSinFunction::GetAmplitude() const {
        return m_amplitude;
    }

    void FrSinFunction::SetHeight(double height) {
        m_amplitude = 0.5*height;
    }

    double FrSinFunction::GetHeight() const {
        return 2. * m_amplitude;
    }

    void FrSinFunction::SetAngularFrequency(double w) {
        assert(w > 0.);
        m_angularFrequency = w;
    }

    double FrSinFunction::GetAngularFrequency() const {
        return m_angularFrequency;
    }

    void FrSinFunction::SetFrequency(double f) {
        assert(f > 0.);
        m_angularFrequency = MU_2PI * f;
    }

    double FrSinFunction::GetFrequency() const {
        return m_angularFrequency / MU_2PI;
    }

    void FrSinFunction::SetPeriod(double T) {
        assert(T > 0.);
        m_angularFrequency = MU_2PI / T;
    }

    double FrSinFunction::GetPeriod() const {
        return MU_2_PI / m_angularFrequency;
    }

    void FrSinFunction::SetPhase(double phase) {
//        assert(0. <= phase <= MU_2_PI); // FIXME : voir pourquoi cet assert ne fonctionne pas !!
        m_phase = phase;
    }

    double FrSinFunction::GetPhase() const {
        return m_phase;
    }

    void FrSinFunction::SetYOffset(double yOffset) {
        m_YOffset = yOffset;
    }

    double FrSinFunction::GetYOffset() const {
        return m_YOffset;
    }

    void FrSinFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;

        double alpha = m_angularFrequency * x + m_phase;
        double salpha = sin(alpha);

        c_y = m_YOffset + salpha;
        c_y_dx = m_angularFrequency * cos(alpha);
        c_y_dxdx = -m_angularFrequency*m_angularFrequency * salpha;
    }

}  // end namespace frydom
