//
// Created by frongere on 12/10/18.
//

#include "FrFunction.h"


namespace frydom {

    // FrFunction_ definitions

    double FrFunction_::GetFunctionValue() const {
        return m_functionValue;
    }

    void FrFunction_::Update(double time) {c_time = time;}



    // FrRamp_ definitions

    void FrRamp_::SetDuration(double duration) {
        m_t1 = m_t0 + duration;
    }

    void FrRamp_::SetIncrease() {
        m_increasing = true;
        Initialize();
    }

    void FrRamp_::SetDecrease() {
        m_increasing = false;
        Initialize();
    }

    void FrRamp_::SetMinVal(double minVal) { m_min = minVal; }

    void FrRamp_::SetMaxVal(double maxVal) { m_max = maxVal; }

    bool FrRamp_::IsActive() {
        return m_active;
    }

    void FrRamp_::Activate() {m_active = true;}

    void FrRamp_::Deactivate() {m_active = false;}

    void FrRamp_::Initialize() {
        double y0, y1;

        if (m_increasing) {
            y0 = m_min;
            y1 = m_max;
        } else {
            y0 = m_max;
            y1 = m_min;
        }
        c_a = (y1 - y0) / (m_t1 - m_t0);
        c_b = y0 - c_a * m_t0;
    }

    void FrRamp_::StepFinalize() {
        if (!m_active) {
            return;
        }

        double y0, y1;
        if (m_increasing) {
            y0 = m_min;
            y1 = m_max;
        } else {
            y0 = m_max;
            y1 = m_min;
        }


        if (c_time < m_t0) {
            m_functionValue = y0;
            return;
        }

        if (c_time <= m_t1) {
            m_functionValue = c_a * c_time + c_b;
            return;
        }

        m_functionValue = y1;


    }
}  // end namespace frydom