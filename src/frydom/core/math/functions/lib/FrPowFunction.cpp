//
// Created by frongere on 11/02/19.
//

#include "FrPowFunction.h"
#include "fmt/format.h"

#include <cmath>


namespace frydom {

    FrPowFunction::FrPowFunction(const frydom::FrFunctionBase &function, double power) : m_power(power) {
        m_function = function.Clone();
    }

    FrPowFunction::FrPowFunction(const FrPowFunction& other) : FrFunctionBase(other) {
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

        double u = m_function->Get_y(x);
        double u_dx = m_function->Get_y_dx(x);
        double u_dxdx = m_function->Get_y_dxdx(x);
        double p_1 = m_power-1;
        double upow = std::pow(u, p_1);

        c_y = std::pow(u, m_power);
        c_y_dx = m_power * u_dx * upow;
        c_y_dxdx = m_power * (u_dxdx * upow + p_1 * u_dx*u_dx * std::pow(u, m_power-2));

    }


    FrPowFunction pow(const FrFunctionBase& function, double power) {
        return FrPowFunction(function, power);
    }

}  // end namespace frydom
