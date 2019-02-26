//
// Created by frongere on 06/02/19.
//

#include "FrLinearRampFunction.h"
#include "frydom/core/math/functions/lib/FrClampFunction.h"
#include "fmt/format.h"

namespace frydom {

    FrLinearRampFunction::FrLinearRampFunction() {
        m_function = clamp_after(clamp_before(FrVarXFunction(), 0), 1).Clone();
    }

    FrLinearRampFunction::FrLinearRampFunction(double x0, double y0, double x1, double y1) : FrFunctionBase() {
        SetByTwoPoints(x0, y0, x1, y1);
    }

    FrLinearRampFunction::FrLinearRampFunction(const frydom::FrLinearRampFunction &other) : FrFunctionBase(other) {}

    FrLinearRampFunction* FrLinearRampFunction::Clone() const {
        return new FrLinearRampFunction(*this);
    }

//    void FrLinearRampFunction::SetY0(double intercept) {
//        m_intercept = intercept;
//    }
//
//    double FrLinearRampFunction::GetY0() const {
//        return m_intercept;
//    }
//
//    void FrLinearRampFunction::SetSlope(double slope) {
//        m_slope = slope;
//    }
//
//    double FrLinearRampFunction::GetSlope() const {
//        return m_slope;
//    }
//
//    void FrLinearRampFunction::SetInterceptAndSlope(double intercept, double slope) {
//        m_intercept = intercept;
//        m_slope = slope;
//    }
//
//    void FrLinearRampFunction::SetXWindow(double x0, double x1) {
//        m_x0 = x0;
//        m_x1 = x1;
//    }
//
    void FrLinearRampFunction::SetByTwoPoints(double x0, double y0, double x1, double y1) {
        double slope = (y1-y0) / (x1-x0);
        double intercept = (y0 - slope*x0);
        m_function = clamp_after(clamp_before((slope*FrVarXFunction() + intercept), x0), x1).Clone();
    }

    void FrLinearRampFunction::Initialize() {

    }

    std::string FrLinearRampFunction::GetRepr() const {
        fmt::MemoryWriter mw;

//        mw.write("Ramp(({:.3g}, {:.3g}), ({:.3g}, {:.3g}))", m_x0, Get_y(m_x0), m_x1, Get_y(m_x1));
//        return mw.str();
        return ""; // TODO
    }

    void FrLinearRampFunction::Eval(double x) const {

        if (IsEval(x)) return;

        c_x = x;
        c_y = m_function->Get_y(x);
        c_y_dx = m_function->Get_y_dx(x);
        c_y_dxdx = m_function->Get_y_dxdx(x);

    }


}  // end namespace frydom
