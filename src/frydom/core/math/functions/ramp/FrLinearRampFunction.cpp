//
// Created by frongere on 06/02/19.
//

#include "FrLinearRampFunction.h"
#include "frydom/core/math/functions/lib/FrClampFunction.h"
#include "fmt/format.h"

namespace frydom {

    FrLinearRampFunction_::FrLinearRampFunction_() {
        m_function = clamp_after(clamp_before(FrVarXFunction(), 0), 1).Clone();
    }

    FrLinearRampFunction_::FrLinearRampFunction_(double x0, double y0, double x1, double y1) : FrFunctionBase() {
        double slope = (y1-y0) / (x1-x0);
        double intercept = (y0 - slope*x0);
        m_function = clamp_after(clamp_before((slope*FrVarXFunction() + intercept), x0), x1).Clone();
    }

    FrLinearRampFunction_::FrLinearRampFunction_(const frydom::FrLinearRampFunction_ &other) : FrFunctionBase(other) {}

    FrLinearRampFunction_* FrLinearRampFunction_::Clone() const {
        return new FrLinearRampFunction_(*this);
    }

//    void FrLinearRampFunction_::SetY0(double intercept) {
//        m_intercept = intercept;
//    }
//
//    double FrLinearRampFunction_::GetY0() const {
//        return m_intercept;
//    }
//
//    void FrLinearRampFunction_::SetSlope(double slope) {
//        m_slope = slope;
//    }
//
//    double FrLinearRampFunction_::GetSlope() const {
//        return m_slope;
//    }
//
//    void FrLinearRampFunction_::SetInterceptAndSlope(double intercept, double slope) {
//        m_intercept = intercept;
//        m_slope = slope;
//    }
//
//    void FrLinearRampFunction_::SetXWindow(double x0, double x1) {
//        m_x0 = x0;
//        m_x1 = x1;
//    }
//
//    void FrLinearRampFunction_::SetByTwoPoints(double x0, double y0, double x1, double y1) {
//        SetSlope((y1-y0) / (x1-x0));
//        SetY0(y0 - GetSlope() * x0);
//        m_x0 = x0;
//        m_x1 = x1;
//    }

    void FrLinearRampFunction_::Initialize() {

    }

    std::string FrLinearRampFunction_::GetRepr() const {
        fmt::MemoryWriter mw;

//        mw.write("Ramp(({:.3g}, {:.3g}), ({:.3g}, {:.3g}))", m_x0, Get_y(m_x0), m_x1, Get_y(m_x1));
//        return mw.str();
        return ""; // TODO
    }

    void FrLinearRampFunction_::Eval(double x) const {

        if (IsEval(x)) return;

        c_x = x;
        c_y = m_function->Get_y(x);
        c_y_dx = m_function->Get_y_dx(x);
        c_y_dxdx = m_function->Get_y_dxdx(x);

    }


}  // end namespace frydom
