//
// Created by frongere on 12/10/18.
//

#include "FrFunction.h"


namespace frydom {

    /*
     * Chrono ChFunction wrapper
     */

    namespace internal {

        FrChronoFunctionWrapper::FrChronoFunctionWrapper(FrFunction_* frydomFunction) :
                m_frydomFunction(frydomFunction) {}

        FrChronoFunctionWrapper* FrChronoFunctionWrapper::Clone() const {
            return new FrChronoFunctionWrapper(m_frydomFunction);
        }

        double FrChronoFunctionWrapper::Get_y(double x) const {
            return m_frydomFunction->Get_y(x);
        }

        double FrChronoFunctionWrapper::Get_y_dx(double x) const {
            return m_frydomFunction->Get_y_dx(x);
        }

        double FrChronoFunctionWrapper::Get_y_dxdx(double x) const {
            return m_frydomFunction->Get_y_dxdx(x);
        }

    }  // end namespace frydom::internal



    /*
     * FrFunction_
     */

    FrFunction_::FrFunction_() {
        m_chronoFunction = std::make_shared<internal::FrChronoFunctionWrapper>(this);
    }

    bool FrFunction_::IsActive() const {
        return m_isActive;
    }

    void FrFunction_::SetActive(bool active) {
        m_isActive = active;
    }

    double FrFunction_::Get_y(double x) const {
        Eval(x);
        return c_y;
    }

    double FrFunction_::Get_y_dx(double x) const {
        Eval(x);
        return c_y_dx;
    }

    double FrFunction_::Get_y_dxdx(double x) const {
        Eval(x);
        return c_y_dxdx;
    }

//    void FrFunction_::Initialize() {
//        Eval(0.);
//    }

    void FrFunction_::StepFinalize() {}

    std::shared_ptr<FrCompositeFunction>
    FrFunction_::operator+(std::shared_ptr<FrFunction_> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::ADD);
    }

    std::shared_ptr<FrNegateFunction> FrFunction_::operator-() {
        return std::make_shared<FrNegateFunction>(this);
    }

    std::shared_ptr<FrCompositeFunction>
    FrFunction_::operator-(std::shared_ptr<FrFunction_> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::SUB);
    }

    std::shared_ptr<FrCompositeFunction>
    FrFunction_::operator*(std::shared_ptr<FrFunction_> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::MUL);
    }

    std::shared_ptr<FrCompositeFunction>
    FrFunction_::operator/(std::shared_ptr<FrFunction_> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::DIV);
    }

    std::shared_ptr<FrCompositeFunction>
    FrFunction_::operator<<(std::shared_ptr<FrFunction_> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::COM);
    }

    std::shared_ptr<FrMultiplyByScalarFunction>
    FrFunction_::operator*(double alpha) {
        return std::make_shared<FrMultiplyByScalarFunction>(this, alpha);
    }

    std::shared_ptr<FrMultiplyByScalarFunction>
    FrFunction_::operator/(double alpha) {
        return std::make_shared<FrMultiplyByScalarFunction>(this, 1./alpha);
    }

    std::shared_ptr<FrMultiplyByScalarFunction> operator*(double alpha, std::shared_ptr<FrFunction_> otherFunction) {
        return (*otherFunction) * alpha;
    }

    std::shared_ptr<FrInverseFunction> operator/(double alpha, std::shared_ptr<FrFunction_> otherFunction) {
        return std::make_shared<FrInverseFunction>(otherFunction.get(), alpha);
    }

    std::shared_ptr<chrono::ChFunction> FrFunction_::GetChronoFunction() {
        return m_chronoFunction;
    }

    double FrFunction_::Estimate_y_dx(double x) const {
        // TODO
    }

    double FrFunction_::Estimate_y_dxdx(double x) const {
        // TODO
    }


    /*
     * FrCompositeFunction
     */

    FrCompositeFunction::FrCompositeFunction(frydom::FrFunction_ *function1, frydom::FrFunction_ *function2,
                                             frydom::FrCompositeFunction::OPERATOR op) :
                                                m_f1(function1),
                                                m_f2(function2),
                                                m_operator(op) {}

    void FrCompositeFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;

        switch (m_operator) {
            case ADD: // (u+v)
                EvalADD();
                break;

            case SUB: // (u-v)
                EvalSUB();
                break;

            case MUL: // (u*v)
                EvalMUL();
                break;

            case DIV: // (u/v)
                EvalDIV();
                break;

            case COM: // (u o v) operator (u << v)
                EvalCOM();
                break;
        }
    }

    void FrCompositeFunction::EvalADD() const {
        c_y      = m_f1->Get_y(c_x) + m_f2->Get_y(c_x);
        c_y_dx   = m_f1->Get_y_dx(c_x) + m_f2->Get_y_dx(c_x);
        c_y_dxdx = m_f1->Get_y_dxdx(c_x) + m_f2->Get_y_dxdx(c_x);
    }

    void FrCompositeFunction::EvalSUB() const {
        c_y = m_f1->Get_y(c_x) - m_f2->Get_y(c_x);
        c_y_dx   = m_f1->Get_y_dx(c_x) - m_f2->Get_y_dx(c_x);
        c_y_dxdx = m_f1->Get_y_dxdx(c_x) - m_f2->Get_y_dxdx(c_x);
    }

    void FrCompositeFunction::EvalMUL() const {
        double u, u_dx, u_dxdx, v, v_dx, v_dxdx;
        EvalFunctions(u, u_dx, u_dxdx, v, v_dx, v_dxdx);

        c_y      = u * v;
        c_y_dx   = u_dx * v + v_dx * u;
        c_y_dxdx = u_dxdx*v + 2.*u_dx*v_dx + v_dxdx*u;

    }

    void FrCompositeFunction::EvalDIV() const {
        double u, u_dx, u_dxdx, v, v_dx, v_dxdx;
        EvalFunctions(u, u_dx, u_dxdx, v, v_dx, v_dxdx);

        c_y = u / v;
        c_y_dx = (u_dx * v - v_dx * u) / (v*v);
        c_y_dxdx = (u_dxdx - 2. * c_y_dx * v_dx - c_y * v_dxdx) / v;
    }

    void FrCompositeFunction::EvalCOM() const {
        double v    = m_f2->Get_y(c_x);
        double v_dx = m_f2->Get_y_dx(c_x);
        double v_dxdx = m_f2->Get_y_dxdx(c_x);

        c_y = m_f1->Get_y(v);
        c_y_dx = v_dx * m_f1->Get_y_dx(v);
        c_y_dxdx = v_dxdx * m_f1->Get_y_dx(v) + v_dx*v_dx * m_f1->Get_y_dxdx(v);
    }


    /*
     * FrNegateFunction
     */

    FrNegateFunction::FrNegateFunction(frydom::FrFunction_ *functionToNegate) : m_functionToNegate(functionToNegate) {}

    void FrNegateFunction::Eval(double x) const {
        if (IsEval(x)) return;
        c_x = x;
        c_y = - m_functionToNegate->Get_y(x);
        c_y_dx = - m_functionToNegate->Get_y_dx(x);
        c_y_dxdx = - m_functionToNegate->Get_y_dxdx(x);

    }

    /*
     * FrMultiplyByScalarFunction
     */

    FrMultiplyByScalarFunction::FrMultiplyByScalarFunction(FrFunction_ *functionToScale, double alpha) :
            m_functionToScale(functionToScale), m_alpha(alpha) {}

    void FrMultiplyByScalarFunction::Eval(double x) const {
        if (IsEval(x)) return;
        c_x = x;
        c_y = m_alpha * m_functionToScale->Get_y(x);
        c_y_dx = m_alpha * m_functionToScale->Get_y_dx(x);
        c_y_dxdx = m_alpha * m_functionToScale->Get_y_dxdx(x);
    }


    /*
     * FrInverseFunction
     */

    FrInverseFunction::FrInverseFunction(FrFunction_ *functionToInverse, double alpha) :
            m_functionToInverse(functionToInverse), m_alpha(alpha) {}

    void FrInverseFunction::Eval(double x) const {
        if (IsEval(x)) return;
        c_x = x;

        double u = m_functionToInverse->Get_y(x);
        double u_dx = m_functionToInverse->Get_y_dx(x);
        double u_dxdx = m_functionToInverse->Get_y_dxdx(x);

        c_y = m_alpha / u;
        c_y_dx = - m_alpha * u_dx / (u*u);
        c_y_dxdx = - m_alpha * (2.*c_y_dx*u_dx + c_y*u_dxdx) / u;

    }




}  // end namespace frydom
