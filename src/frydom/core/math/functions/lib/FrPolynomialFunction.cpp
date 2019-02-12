//
// Created by frongere on 11/02/19.
//

#include "FrPolynomialFunction.h"

#include "FrPowFunction.h"


namespace frydom {


    FrPolynomialFunction::FrPolynomialFunction(double constant) {
        m_function = FrConstantFunction(constant).Clone();
    }

    FrPolynomialFunction::FrPolynomialFunction(const FrPolynomialFunction &other) : FrFunctionBase(other) {}

    FrPolynomialFunction *FrPolynomialFunction::Clone() const {
        return new FrPolynomialFunction(*this);
    }

    void FrPolynomialFunction::Add(double val, unsigned int power) {
        assert(power > 0);
        *this += val * pow(FrVarXFunction(), power);
    }

    std::string FrPolynomialFunction::GetRepr() const {
        return ""; // TODO
    }

    void FrPolynomialFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        c_y = m_function->Get_y(x);
        c_y_dx = m_function->Get_y_dx(x);
        c_y_dxdx = m_function->Get_y_dxdx(x);
    }



}  // end namespace frydom
