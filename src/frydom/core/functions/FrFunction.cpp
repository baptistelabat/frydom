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




//    namespace internal {
//
//        FrFunctionWrapper::FrFunctionWrapper(frydom::FrFunction_ *frydomFunction) : m_frydomFunction(frydomFunction) {}
//
//        double FrFunctionWrapper::Get_y(double x) const {
//            return ChFunction::Get_y(x);
//        }
//
//        double FrFunctionWrapper::Get_y_dx(double x) const {
//            return ChFunction::Get_y_dx(x);
//        }
//
//        double FrFunctionWrapper::Get_y_dxdx(double x) const {
//            return ChFunction::Get_y_dxdx(x);
//        }
//
//
//    }  // end namespace frydom::internal






//    FrFunction_::FrFunction_() {
//
//    }
//
//    double FrFunctionBase::Get_y(double x) const {
//
//    }
//
//    double FrFunctionBase::Get_y_dx(double x) const {
//
//    }
//
//    double FrFunctionBase::Get_y_dxdx(double x) const {
//
//    }

    bool FrFunctionBase::IsActive() const {
        return m_isActive;
    }

    void FrFunctionBase::SetActive(bool active) {
        m_isActive = active;
    }


    std::shared_ptr<FrFunctionBase>
    FrFunctionBase::operator+(const std::shared_ptr<FrFunctionBase> &otherFunction) const {
        std::cout << "On additione des fonctions" << std::endl;

        auto newFunction = std::make_shared<FrCompositeFunction>();
        newFunction->m_function1 = shared_from_this();



    }

    std::shared_ptr<FrFunctionBase> FrFunctionBase::operator-() const {

    }

    std::shared_ptr<FrFunctionBase>
    FrFunctionBase::operator-(const std::shared_ptr<FrFunctionBase> &otherFunction) const {

    }

    std::shared_ptr<FrFunctionBase>
    FrFunctionBase::operator*(const std::shared_ptr<FrFunctionBase> &otherFunction) const {

    }

    std::shared_ptr<FrFunctionBase>
    FrFunctionBase::operator/(const std::shared_ptr<FrFunctionBase> &otherFunction) const {

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





    double FrCompositeFunction::Get_y(double x) const {
        switch (m_operator) {
            case ADD:
                return m_function1->Get_y(x) + m_function2->Get_y(x);
            case SUB:
                return m_function1->Get_y(x) - m_function2->Get_y(x);
            case MUL:
                return m_function1->Get_y(x) * m_function2->Get_y(x);
            case DIV:
                return m_function1->Get_y(x) / m_function2->Get_y(x);
        }
    }

    double FrCompositeFunction::Get_y_dx(double x) const {
        double u, v;
        double u_dx = m_function1->Get_y_dx(x);
        double v_dx = m_function2->Get_y_dx(x);

        switch (m_operator) {
            case ADD:
                return u_dx + v_dx;
            case SUB:
                return u_dx - v_dx;
            case MUL:
                u = m_function1->Get_y(x);
                v = m_function2->Get_y(x);
                return u_dx * v + v_dx * u;
            case DIV:
                u = m_function1->Get_y(x);
                v = m_function2->Get_y(x);
                return (u_dx * v - v_dx * u) / (v*v);
        }
    }

    double FrCompositeFunction::Get_y_dxdx(double x) const {

        double u, v, u_dx, v_dx;

        double u_dxdx = m_function1->Get_y_dxdx(x);
        double v_dxdx = m_function2->Get_y_dxdx(x);


        switch (m_operator) {
            case ADD:
                return u_dxdx + v_dxdx;
            case SUB:
                return u_dxdx - v_dxdx;
            case MUL:
                u = m_function1->Get_y(x);
                v = m_function2->Get_y(x);

                u_dx = m_function1->Get_y_dx(x);
                v_dx = m_function2->Get_y_dx(x);
                return (u_dx * v - v_dx * u) / (v*v);

            case DIV:
                u = m_function1->Get_y(x);
                v = m_function2->Get_y(x);

                u_dx = m_function1->Get_y_dx(x);
                v_dx = m_function2->Get_y_dx(x);

                double f = u / v;
                double f_dx = (u_dx * v - u * v_dx) / (v*v);
                double f_dxdx = (u_dxdx - 2. * f_dx * v_dx - f * v_dxdx) / v;

                return f_dxdx;
        }
    }












}  // end namespace frydom
