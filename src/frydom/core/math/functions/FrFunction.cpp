//
// Created by frongere on 12/10/18.
//

#include "FrFunction.h"


namespace frydom {



//    // FrFunction_ definitions
//
//    double FrFunction_::GetFunctionValue() const {
//        return m_functionValue;
//    }
//
//    void FrFunction_::Update(double time) {c_time = time;}
//
//
//
//
//
//
//
//
//
//
//    // FrRamp_ definitions
//
//    void FrRamp_::SetDuration(double duration) {
//        m_t1 = m_t0 + duration;
//    }
//
//    void FrRamp_::SetIncrease() {
//        m_increasing = true;
//        Initialize();
//    }
//
//    void FrRamp_::SetDecrease() {
//        m_increasing = false;
//        Initialize();
//    }
//
//    void FrRamp_::SetMinTime(double minTime) { m_t0 = minTime;}
//
//    void FrRamp_::SetMaxTime(double maxTime) { m_t1 = maxTime;}
//
//    void FrRamp_::SetMinVal(double minVal) { m_min = minVal; }
//
//    void FrRamp_::SetMaxVal(double maxVal) { m_max = maxVal; }
//
//    bool FrRamp_::IsActive() {
//        return m_active;
//    }
//
//    void FrRamp_::Activate() {m_active = true;}
//
//    void FrRamp_::Deactivate() {m_active = false;}
//
//    void FrRamp_::Initialize() {
//        assert(m_min<=m_max);
//        double y0, y1;
//
//        if (m_increasing) {
//            y0 = m_min;
//            y1 = m_max;
//        } else {
//            y0 = m_max;
//            y1 = m_min;
//        }
//        c_a = (y1 - y0) / (m_t1 - m_t0);
//        c_b = y0 - c_a * m_t0;
//    }
//
//    void FrRamp_::StepFinalize() {
//        if (!m_active) {
//            return;
//        }
//
//        double y0, y1;
//        if (m_increasing) {
//            y0 = m_min;
//            y1 = m_max;
//        } else {
//            y0 = m_max;
//            y1 = m_min;
//        }
//
//
//        if (c_time < m_t0) {
//            m_functionValue = y0;
//            return;
//        }
//
//        if (c_time <= m_t1) {
//            m_functionValue = c_a * c_time + c_b;
//            return;
//        }
//
//        m_functionValue = y1;
//
//
//    }

















    ///////////////// REFACTORING


    bool FrBaseFunction::IsActive() const {
        return m_isActive;
    }

    void FrBaseFunction::SetActive(bool active) {
        m_isActive = active;
    }


    std::shared_ptr<FrCompositeFunction>
    FrBaseFunction::operator+(std::shared_ptr<FrBaseFunction> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::ADD);
    }

    std::shared_ptr<FrNegateFunction> FrBaseFunction::operator-() {
        return std::make_shared<FrNegateFunction>(this);
    }

    std::shared_ptr<FrCompositeFunction>
    FrBaseFunction::operator-(std::shared_ptr<FrBaseFunction> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::SUB);
    }

    std::shared_ptr<FrCompositeFunction>
    FrBaseFunction::operator*(std::shared_ptr<FrBaseFunction> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::MUL);
    }

    std::shared_ptr<FrCompositeFunction>
    FrBaseFunction::operator/(std::shared_ptr<FrBaseFunction> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::DIV);
    }

    std::shared_ptr<FrCompositeFunction>
    FrBaseFunction::operator<<(std::shared_ptr<FrBaseFunction> otherFunction) {
        return std::make_shared<FrCompositeFunction>(this, otherFunction.get(), FrCompositeFunction::COM);
    }

    std::shared_ptr<FrMultiplyByScalarFunction>
    FrBaseFunction::operator*(double alpha) {
        return std::make_shared<FrMultiplyByScalarFunction>(this, alpha);
    }

    std::shared_ptr<FrMultiplyByScalarFunction>
    FrBaseFunction::operator/(double alpha) {
        return std::make_shared<FrMultiplyByScalarFunction>(this, 1/alpha);
    }

    std::shared_ptr<FrMultiplyByScalarFunction> operator*(double alpha, std::shared_ptr<FrBaseFunction> otherFunction) {
        return (*otherFunction) * alpha;
    }

    std::shared_ptr<FrInverseFunction> operator/(double alpha, std::shared_ptr<FrBaseFunction> otherFunction) {
        return std::make_shared<FrInverseFunction>(otherFunction.get(), alpha);
    }






    double FrFunction_::Get_y(double x) const {
        return m_chronoFunction->Get_y(x);
    }

    double FrFunction_::Get_y_dx(double x) const {
        return m_chronoFunction->Get_y_dx(x);
    }

    double FrFunction_::Get_y_dxdx(double x) const {
        return m_chronoFunction->Get_y_dxdx(x);
    }



    FrCompositeFunction::FrCompositeFunction(frydom::FrBaseFunction *function1, frydom::FrBaseFunction *function2,
                                             frydom::FrCompositeFunction::OPERATOR op) :
                                                m_f1(function1),
                                                m_f2(function2),
                                                m_operator(op) {}

    double FrCompositeFunction::Get_y(double x) const { // TODO : inserer tu try pour DIV pour les divisions par 0 et autres...
        switch (m_operator) {
            case ADD: // (u+v)
                return m_f1->Get_y(x) + m_f2->Get_y(x);

            case SUB: // (u-v)
                return m_f1->Get_y(x) - m_f2->Get_y(x);

            case MUL: // (u*v)
                return m_f1->Get_y(x) * m_f2->Get_y(x);

            case DIV: // (u/v)
                return m_f1->Get_y(x) / m_f2->Get_y(x);

            case COM: // (u o v)
                return m_f1->Get_y(m_f2->Get_y(x));
        }
    }

    double FrCompositeFunction::Get_y_dx(double x) const { // TODO : inserer tu try pour DIV pour les divisions par 0 et autres...
        double u, v;
        double u_dx = m_f1->Get_y_dx(x);
        double v_dx = m_f2->Get_y_dx(x);

        switch (m_operator) {
            case ADD: // (u+v)'
                return u_dx + v_dx;

            case SUB: // (u-v)'
                return u_dx - v_dx;

            case MUL: // (u*v)'
                u = m_f1->Get_y(x);
                v = m_f2->Get_y(x);
                return u_dx * v + v_dx * u;

            case DIV: // (u/v)'
                u = m_f1->Get_y(x);
                v = m_f2->Get_y(x);
                return (u_dx * v - v_dx * u) / (v*v);

            case COM: // (u o v)'
                v = m_f2->Get_y(x);
                return v_dx * m_f1->Get_y_dx(v);
        }
    }

    double FrCompositeFunction::Get_y_dxdx(double x) const { // TODO : inserer tu try pour DIV pour les divisions par 0 et autres...

        double u, v, u_dx, v_dx, f, f_dx, f_dxdx;

        double u_dxdx = m_f1->Get_y_dxdx(x);
        double v_dxdx = m_f2->Get_y_dxdx(x);


        switch (m_operator) {
            case ADD: // (u+v)''
                return u_dxdx + v_dxdx;

            case SUB: // (u-v)''
                return u_dxdx - v_dxdx;

            case MUL: // (u*v)''
                u = m_f1->Get_y(x);
                v = m_f2->Get_y(x);

                u_dx = m_f1->Get_y_dx(x);
                v_dx = m_f2->Get_y_dx(x);

                return (u_dx * v - v_dx * u) / (v*v);

            case DIV: // (u/v)''
                u = m_f1->Get_y(x);
                v = m_f2->Get_y(x);

                u_dx = m_f1->Get_y_dx(x);
                v_dx = m_f2->Get_y_dx(x);

                f = u / v;
                f_dx = (u_dx * v - u * v_dx) / (v*v);
                f_dxdx = (u_dxdx - 2. * f_dx * v_dx - f * v_dxdx) / v;

                return f_dxdx;

            case COM: // (u o v)''
                v = m_f2->Get_y(x);
                v_dx = m_f2->Get_y_dx(x);

                return v_dxdx * m_f1->Get_y_dx(v) + v_dx*v_dx * m_f1->Get_y_dxdx(v);

        }
    }



    FrNegateFunction::FrNegateFunction(frydom::FrBaseFunction *functionToNegate) : m_functionToNegate(functionToNegate) {}

    double FrNegateFunction::Get_y(double x) const {
        return - m_functionToNegate->Get_y(x);
    }

    double FrNegateFunction::Get_y_dx(double x) const {
        return - m_functionToNegate->Get_y_dx(x);
    }

    double FrNegateFunction::Get_y_dxdx(double x) const {
        return - m_functionToNegate->Get_y_dxdx(x);
    }





    FrMultiplyByScalarFunction::FrMultiplyByScalarFunction(FrBaseFunction *functionToScale, double alpha) :
            m_functionToScale(functionToScale), m_alpha(alpha) {}

    double FrMultiplyByScalarFunction::Get_y(double x) const {
        return m_alpha * m_functionToScale->Get_y(x);
    }

    double FrMultiplyByScalarFunction::Get_y_dx(double x) const {
        return m_alpha * m_functionToScale->Get_y_dx(x);
    }

    double FrMultiplyByScalarFunction::Get_y_dxdx(double x) const {
        return m_alpha * m_functionToScale->Get_y_dxdx(x);
    }




    FrInverseFunction::FrInverseFunction(FrBaseFunction *functionToInverse, double alpha) :
            m_functionToInverse(functionToInverse), m_alpha(alpha) {}

    double FrInverseFunction::Get_y(double x) const {
        return m_functionToInverse->Get_y(x) / m_alpha;
    }

    double FrInverseFunction::Get_y_dx(double x) const {
        return m_functionToInverse->Get_y_dx(x) / m_alpha;
    }

    double FrInverseFunction::Get_y_dxdx(double x) const {
        return m_functionToInverse->Get_y_dxdx(x) / m_alpha;
    }


}  // end namespace frydom
