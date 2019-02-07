//
// Created by frongere on 06/02/19.
//

#include "FrRamp.h"


namespace frydom {

    FrRamp_::FrRamp_() : FrFunction_() {
//        Initialize();
    }

    void FrRamp_::SetY0(double intercept) {
        m_intercept = intercept;
    }

    double FrRamp_::GetY0() const {
        return m_intercept;
    }

    void FrRamp_::SetSlope(double slope) {
        m_slope = slope;
    }

    double FrRamp_::GetSlope() const {
        return m_slope;
    }

    void FrRamp_::Set(double intercept, double slope) {
        m_intercept = intercept;
        m_slope = slope;
    }

    void FrRamp_::SetIsWindowed(bool limit) {
        m_isWindowed = limit;
        if (limit) Eval(c_x);
    }

    bool FrRamp_::GetIsLimited() const {
        return m_isWindowed;
    }

    void FrRamp_::SetXWindow(double xmin, double xmax) {
        m_xmin = xmin;
        m_xmax = xmax;
        m_isWindowed = true;
        Eval(c_x);
    }

    void FrRamp_::SetByTwoPoints(double xmin, double ymin, double xmax, double ymax, bool isWindowed) {
        SetSlope((ymax-ymin) / (xmax-xmin));
        SetY0(ymin - GetSlope() * xmin);
        // TODO : utiliser isWindowed
    }

    void FrRamp_::Initialize() {

    }

    void FrRamp_::Eval(double x) const {
        if (IsEval(x)) return;
        c_x = x;

        c_y = m_slope * x + m_intercept;
        c_y_dx = m_slope;
        c_y_dxdx = 0.;

    }


}  // end namespace frydom
