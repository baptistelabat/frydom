//
// Created by frongere on 11/02/19.
//

#include "FrPowFunction.h"
#include "fmt/format.h"


namespace frydom {

    FrPowFunction::FrPowFunction(double power) : m_power(power) {}

    FrPowFunction::FrPowFunction(const FrPowFunction& other) {
        m_power = other.m_power;
    }

    FrPowFunction* FrPowFunction::Clone() const {
        return new FrPowFunction(*this);
    }

    void FrPowFunction::Set(double power) {
        m_power = power;
    }

    std::string FrPowFunction::GetRepr() const {
        fmt::MemoryWriter mw;
//        mw <<
        return "";
    }

    void FrPowFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        c_y = pow(x, m_power);
        double p_1 = m_power-1;
        c_y_dx = m_power * pow(x, p_1);
        c_y_dxdx = m_power * p_1 * pow(x, m_power-2);

    }

}  // end namespace frydom
