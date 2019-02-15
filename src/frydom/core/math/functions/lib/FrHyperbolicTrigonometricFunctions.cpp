//
// Created by frongere on 12/02/19.
//

#include "FrHyperbolicTrigonometricFunctions.h"


namespace frydom {


    /*
     * FrCosFunction
     */

    FrCosHFunction::FrCosHFunction(double alpha) {
        m_function = FrConstantFunction(alpha).Clone();
    }

    FrCosHFunction::FrCosHFunction(const FrFunctionBase &function) {
        m_function = function.Clone();
    }

    FrCosHFunction::FrCosHFunction(const FrCosHFunction &other) : FrFunctionBase(other) {}

    FrCosHFunction *FrCosHFunction::Clone() const {
        return new FrCosHFunction(*this);
    }

    std::string FrCosHFunction::GetRepr() const {
        return ""; // TODO
    }

    void FrCosHFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double u = m_function->Get_y(x);
        double u_dx = m_function->Get_y_dx(x);
        double u_dxdx = m_function->Get_y_dxdx(x);
        double ch = std::cosh(u);
        double sh = std::sinh(u);

        c_y = ch;
        c_y_dx = u_dx * sh;
        c_y_dxdx = u_dxdx * sh + u_dx*u_dx * ch;

    }

    FrCosHFunction cosh(const FrFunctionBase &function) {
        return FrCosHFunction(function);
    }


    /*
     * FrSinFunction
     */

    FrSinHFunction::FrSinHFunction(double alpha) {
        m_function = FrConstantFunction(alpha).Clone();
    }

    FrSinHFunction::FrSinHFunction(const FrFunctionBase &function) {
        m_function = function.Clone();
    }

    FrSinHFunction::FrSinHFunction(const FrSinHFunction &other) : FrFunctionBase(other) {}

    FrSinHFunction *FrSinHFunction::Clone() const {
        return new FrSinHFunction(*this);
    }

    std::string FrSinHFunction::GetRepr() const {
        return ""; // TODO
    }

    void FrSinHFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double u = m_function->Get_y(x);
        double u_dx = m_function->Get_y_dx(x);
        double u_dxdx = m_function->Get_y_dxdx(x);
        double ch = std::cosh(u);
        double sh = std::sinh(u);

        c_y = sh;
        c_y_dx = u_dx * ch;
        c_y_dxdx = u_dxdx * ch + u_dx*u_dx * sh;

    }

    FrSinHFunction sinh(const FrFunctionBase &function) {
        return FrSinHFunction(function);
    }


    /*
     * FrTanFunction
     */

    FrTanHFunction::FrTanHFunction(double alpha) {
        m_function = FrConstantFunction(alpha).Clone();
    }

    FrTanHFunction::FrTanHFunction(const FrFunctionBase &function) {
        m_function = function.Clone();
    }

    FrTanHFunction::FrTanHFunction(const FrTanHFunction &other) : FrFunctionBase(other) {}

    FrTanHFunction *FrTanHFunction::Clone() const {
        return new FrTanHFunction(*this);
    }

    std::string FrTanHFunction::GetRepr() const {
        return ""; // TODO
    }

    void FrTanHFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;

        double u = m_function->Get_y(x);
        double u_dx = m_function->Get_y_dx(x);
        double u_dxdx = m_function->Get_y_dxdx(x);

        double th = std::tanh(u);
        double tmp = 1. - th*th;

        c_y = th;
        c_y_dx = u_dx * tmp;
        c_y_dxdx = u_dxdx * tmp - 2.*u_dx*c_y*c_y_dx;

    }

    FrTanHFunction tanh(const FrFunctionBase &function) {
        return FrTanHFunction(function);
    }


    /*
     * Invers trigonometric functions
     */

    FrACosHFunction::FrACosHFunction(double alpha) {
        m_function = FrConstantFunction(alpha).Clone();
    }

    FrACosHFunction::FrACosHFunction(const FrFunctionBase &function) {
        m_function = function.Clone();
    }

    FrACosHFunction::FrACosHFunction(const FrACosHFunction &other) : FrFunctionBase(other) {}

    FrACosHFunction *FrACosHFunction::Clone() const {
        return new FrACosHFunction(*this);
    }

    std::string FrACosHFunction::GetRepr() const {
        return ""; // TODO
    }

    void FrACosHFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double u = m_function->Get_y(x);
        double u_dx = m_function->Get_y_dx(x);
        double u_dxdx = m_function->Get_y_dxdx(x);
        double tmp = u*u - 1.;
        double stmp = sqrt(tmp);

        c_y = std::acosh(u);
        c_y_dx = u_dx / stmp;
        c_y_dxdx = u_dxdx / stmp - u_dx*u_dx * u / std::pow(tmp, 1.5);

    }

    FrACosHFunction acosh(const FrFunctionBase &function) {
        return FrACosHFunction(function);
    }


    /*
     * FrSinFunction
     */

    FrASinHFunction::FrASinHFunction(double alpha) {
        m_function = FrConstantFunction(alpha).Clone();
    }

    FrASinHFunction::FrASinHFunction(const FrFunctionBase &function) {
        m_function = function.Clone();
    }

    FrASinHFunction::FrASinHFunction(const FrASinHFunction &other) : FrFunctionBase(other) {}

    FrASinHFunction *FrASinHFunction::Clone() const {
        return new FrASinHFunction(*this);
    }

    std::string FrASinHFunction::GetRepr() const {
        return ""; // TODO
    }

    void FrASinHFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double u = m_function->Get_y(x);
        double u_dx = m_function->Get_y_dx(x);
        double u_dxdx = m_function->Get_y_dxdx(x);
        double tmp = u*u + 1.;
        double stmp = sqrt(tmp);

        c_y = std::acosh(u);
        c_y_dx = u_dx / stmp;
        c_y_dxdx = u_dxdx / stmp - u_dx*u_dx * u / std::pow(tmp, 1.5);

    }

    FrASinHFunction asinh(const FrFunctionBase &function) {
        return FrASinHFunction(function);
    }


    /*
     * FrTanFunction
     */

    FrATanHFunction::FrATanHFunction(double alpha) {
        m_function = FrConstantFunction(alpha).Clone();
    }

    FrATanHFunction::FrATanHFunction(const FrFunctionBase &function) {
        m_function = function.Clone();
    }

    FrATanHFunction::FrATanHFunction(const FrATanHFunction &other) : FrFunctionBase(other) {}

    FrATanHFunction *FrATanHFunction::Clone() const {
        return new FrATanHFunction(*this);
    }

    std::string FrATanHFunction::GetRepr() const {
        return ""; // TODO
    }

    void FrATanHFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double u = m_function->Get_y(x);
        double u_dx = m_function->Get_y_dx(x);
        double u_dxdx = m_function->Get_y_dxdx(x);
        double tmp = 1 - u*u;

        c_y = std::atanh(u);
        c_y_dx = u_dx / tmp;
        c_y_dxdx = u_dxdx / tmp + 2. * u_dx*u_dx * u / (tmp*tmp);

    }

    FrATanHFunction atanh(const FrFunctionBase &function) {
        return FrATanHFunction(function);
    }

}  // end namespace frydom
