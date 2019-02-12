//
// Created by frongere on 11/02/19.
//

#include "FrSaturateFunction.h"
#include "fmt/format.h"


namespace frydom {

    FrSaturateFunction::FrSaturateFunction(const FrFunctionBase& function) {
        m_function = function.Clone();
    }

    FrSaturateFunction::FrSaturateFunction(const FrSaturateFunction& other) {
        m_ymin = other.m_ymin;
        m_ymax = other.m_ymax;
    }

    FrSaturateFunction* FrSaturateFunction::Clone() const {
        return new FrSaturateFunction(*this);
    }

    void FrSaturateFunction::SetYMin(double ymin) {
        m_ymin = ymin;
    }

    void FrSaturateFunction::SetYMax(double ymax) {
        m_ymax = ymax;
    }

    std::string FrSaturateFunction::GetRepr() const {
        fmt::MemoryWriter mw;
//        mw <<
        return "";  // TODO
    }

    void FrSaturateFunction::Eval(double x) const {

        c_x = x;
        c_y = m_function->Get_y(x);
        c_y_dx = m_function->Get_y_dx(x);
        c_y_dxdx = m_function->Get_y_dxdx(x);

        if (c_y <= m_ymin) {
            c_y = m_ymin;
            c_y_dx = 0.;
            c_y_dxdx = 0.;
        }

        if (c_y >= m_ymax) {
            c_y = m_ymax;
            c_y_dx = 0.;
            c_y_dxdx = 0.;
        }

    }


}  // end namespace frydom
