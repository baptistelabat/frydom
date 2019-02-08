//
// Created by frongere on 06/02/19.
//

#include "FrLinearRampFunction.h"


namespace frydom {

    FrRampFunction_::FrRampFunction_() : FrFunction_() {
//        Initialize();
    }

    void FrRampFunction_::SetY0(double intercept) {
        m_intercept = intercept;
    }

    double FrRampFunction_::GetY0() const {
        return m_intercept;
    }

    void FrRampFunction_::SetSlope(double slope) {
        m_slope = slope;
    }

    double FrRampFunction_::GetSlope() const {
        return m_slope;
    }

    void FrRampFunction_::SetInterceptAndSlope(double intercept, double slope) {
        m_intercept = intercept;
        m_slope = slope;
    }

    void FrRampFunction_::SetXWindow(double x0, double x1) {
        m_x0 = x0;
        m_x1 = x1;
    }

    void FrRampFunction_::SetByTwoPoints(double x0, double y0, double x1, double y1) {
        SetSlope((y1-y0) / (x1-x0));
        SetY0(y0 - GetSlope() * x0);
        m_x0 = x0;
        m_x1 = x1;
    }

    void FrRampFunction_::Initialize() {

    }

    void FrRampFunction_::Eval(double x) const {

        double xTmp(x);
        if (xTmp <= m_x0) {
            xTmp = m_x0;
            c_y_dx = 0.;
        } else if (xTmp >= m_x1) {
            xTmp = m_x1;
            c_y_dx = 0.;
        } else {
            c_y_dx = m_slope;
        }

        if (IsEval(xTmp)) return;

        c_x = xTmp;

        c_y = m_slope * xTmp + m_intercept;
        c_y_dxdx = 0.;

    }


}  // end namespace frydom
