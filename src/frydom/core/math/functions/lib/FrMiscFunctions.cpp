//
// Created by frongere on 12/02/19.
//

#include "FrMiscFunctions.h"
#include "MathUtils/MathUtils.h"


namespace frydom {


    /*
     * FrSqrtFunction
     */

    FrSqrtFunction::FrSqrtFunction(double alpha) {
        m_function = FrConstantFunction(alpha).Clone();
    }

    FrSqrtFunction::FrSqrtFunction(const FrFunctionBase &function) {
        m_function = function.Clone();
    }

    FrSqrtFunction::FrSqrtFunction(const FrSqrtFunction &other) : FrFunctionBase(other) {}

    FrSqrtFunction *FrSqrtFunction::Clone() const {
        return new FrSqrtFunction(*this);
    }

    std::string FrSqrtFunction::GetRepr() const {
        return ""; // TODO
    }

    void FrSqrtFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double u = m_function->Get_y(x);
        double u_dx = m_function->Get_y_dx(x);
        double u_dxdx = m_function->Get_y_dxdx(x);
        double sqrtu = std::sqrt(u);

        c_y = sqrtu;
        c_y_dx = 0.5 * u_dx / sqrtu;
        c_y_dxdx = 0.5 * u_dxdx / sqrtu - 0.25 * u_dx*u_dx / std::pow(u, 1.5);

    }

    FrSqrtFunction sqrt(const FrFunctionBase &function) {
        return FrSqrtFunction(function);
    }

    /*
     * FrAbsFunction
     */

    FrAbsFunction::FrAbsFunction(double alpha) {
        m_function = FrConstantFunction(alpha).Clone();
    }

    FrAbsFunction::FrAbsFunction(const FrFunctionBase &function) {
        m_function = function.Clone();
    }

    FrAbsFunction::FrAbsFunction(const FrAbsFunction &other) : FrFunctionBase(other) {}

    FrAbsFunction *FrAbsFunction::Clone() const {
        return new FrAbsFunction(*this);
    }

    std::string FrAbsFunction::GetRepr() const {
        return ""; // TODO
    }

    void FrAbsFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double u = m_function->Get_y(x);
        double u_dx = m_function->Get_y_dx(x);
        double u_dxdx = m_function->Get_y_dxdx(x);

        double signum = std::copysign(1., u);

        c_y = fabs(u);
        c_y_dx = u_dx * signum;
        c_y_dxdx = u_dxdx * signum;

    }

    FrAbsFunction abs(const FrFunctionBase &function) {
        return FrAbsFunction(function);
    }

//    /*
//     * FrSignFunction
//     */
//
//    FrSignFunction::FrSignFunction(double alpha) {
//        m_function = FrConstantFunction(alpha).Clone();
//    }
//
//    FrSignFunction::FrSignFunction(const FrFunctionBase &function) {
//        m_function = function.Clone();
//    }
//
//    FrSignFunction::FrSignFunction(const FrSignFunction &other) : FrFunctionBase(other) {}
//
//    FrSignFunction *FrSignFunction::Clone() const {
//        return new FrSignFunction(*this);
//    }
//
//    std::string FrSignFunction::GetRepr() const {
//        return ""; // TODO
//    }
//
//    void FrSignFunction::Eval(double x) const {
//        if (IsEval(x)) return;
//
//        c_x = x;
////        double u = m_function->Get_y(x);
////        double u_dx = m_function->Get_y_dx(x);
////        double u_dxdx = m_function->Get_y_dxdx(x);
////        double ch = std::cosh(u);
////        double sh = std::sinh(u);
////
////        c_y = ch;
////        c_y_dx = u_dx * sh;
////        c_y_dxdx = u_dxdx * sh + u_dx*u_dx * ch;
//
//    }
//
//    FrSignFunction sign(const FrFunctionBase &function) {
//        return FrSignFunction(function);
//    }





}  // end namespace frydom
