//
// Created by frongere on 12/10/18.
//

#include "FrFunction.h"

#include "fmt/format.h"


namespace frydom {

    /*
     * Chrono ChFunction wrapper
     */

    namespace internal {

        FrChronoFunctionWrapper::FrChronoFunctionWrapper(FrFunctionBase* frydomFunction) :
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

    FrFunctionBase::FrFunctionBase() {
        m_chronoFunction = std::make_shared<internal::FrChronoFunctionWrapper>(this);
    }

//    FrFunctionBase::FrFunctionBase(const frydom::FrCompositeFunction &other) {
//        this = other.Clone();
//    }

    bool FrFunctionBase::IsActive() const {
        return m_isActive;
    }

    void FrFunctionBase::SetActive(bool active) {
        m_isActive = active;
    }

    void FrFunctionBase::SetXOffset(double xOffset) {
        m_xOffset = xOffset;
    }

    double FrFunctionBase::GetXOffset() const {
        return m_xOffset;
    }

    double FrFunctionBase::Get_y(double x) const {
        Eval(x - m_xOffset);
        return c_y;
    }

    double FrFunctionBase::Get_y_dx(double x) const {
        Eval(x - m_xOffset);
        return c_y_dx;
    }

    double FrFunctionBase::Get_y_dxdx(double x) const {
        Eval(x - m_xOffset);
        return c_y_dxdx;
    }

    void FrFunctionBase::StepFinalize() {}

    double FrFunctionBase::operator()(double x) const {
        return Get_y(x);
    }

    std::shared_ptr<chrono::ChFunction> FrFunctionBase::GetChronoFunction() {
        return m_chronoFunction;
    }

    void FrFunctionBase::WriteToGnuPlotFile(double xmin, double xmax, double dx, std::string filename) const {

        fmt::MemoryWriter mw;

        mw << "#x\ty\tdy\tdydy\n";
        double x(xmin);
        while (x <= xmax) {
            mw << x << "\t"
               << Get_y(x) << "\t"
               << Get_y_dx(x) << "\t"
               << Get_y_dxdx(x) << "\n";
            x += dx;
        }

        // Writing data file
        std::ofstream dataFile;
        dataFile.open(filename+".dat", std::ios::trunc);
        dataFile << mw.str();
        dataFile.close();
        mw.clear();

        // Writing gnuplot file
        mw.write("set grid\n");
        mw.write("plot \"{:s}.dat\" using 1:2 with lines title \"y\", ", filename);
        mw.write("\"{:s}.dat\" using 1:3 with lines title \"dy\", ", filename);
        mw.write("\"{:s}.dat\" using 1:4 with lines title \"dydy\"\n", filename);

        std::ofstream gnuFile;
        gnuFile.open(filename+".gnuplot", std::ios::trunc);
        gnuFile << mw.str();
        gnuFile.close();

    }

    double FrFunctionBase::Estimate_y_dx(double x) const {
        // TODO
    }

    double FrFunctionBase::Estimate_y_dxdx(double x) const {
        // TODO
    }


    /*
     * Operators
     */

    FrCompositeFunction FrFunctionBase::operator+(const FrFunctionBase& other) {
        return FrCompositeFunction(*this, other, FrCompositeFunction::OPERATOR::ADD);
    }

    FrCompositeFunction FrFunctionBase::operator-(const FrFunctionBase& other) {
        return FrCompositeFunction(*this, other, FrCompositeFunction::OPERATOR::SUB);
    }

    FrCompositeFunction FrFunctionBase::operator*(const FrFunctionBase& other) {
        return FrCompositeFunction(*this, other, FrCompositeFunction::OPERATOR::MUL);
    }

    FrCompositeFunction FrFunctionBase::operator/(const FrFunctionBase& other) {
        return FrCompositeFunction(*this, other, FrCompositeFunction::OPERATOR::DIV);
    }

    FrCompositeFunction FrFunctionBase::operator<<(const FrFunctionBase& other) {
        return FrCompositeFunction(*this, other, FrCompositeFunction::OPERATOR::COM);
    }

    FrNegateFunction FrFunctionBase::operator-() {
        return FrNegateFunction(*this);
    }

    FrMultiplyByScalarFunction FrFunctionBase::operator*(double alpha) {
        return FrMultiplyByScalarFunction(*this, alpha);
    }

    FrMultiplyByScalarFunction FrFunctionBase::operator/(double alpha) {
        return FrMultiplyByScalarFunction(*this, 1./alpha);
    }

    FrMultiplyByScalarFunction operator*(double alpha, const FrFunctionBase& functionToScale) {
        return FrMultiplyByScalarFunction(functionToScale, alpha);
    }

    FrAddScalarToFunction FrFunctionBase::operator+(double alpha) {
        return FrAddScalarToFunction(*this, alpha);
    }

    FrAddScalarToFunction FrFunctionBase::operator-(double alpha) {
        return FrAddScalarToFunction(*this, -alpha);
    }

    FrAddScalarToFunction operator+(double alpha, const FrFunctionBase& functionToAddScalar) {
        return FrAddScalarToFunction(functionToAddScalar, alpha);
    }

    FrInverseFunction operator/(double alpha, const FrFunctionBase& functionToInverse) {
        return FrInverseFunction(functionToInverse, alpha);
    }

//    FrPowerFunction pow(const FrFunctionBase& functionToPow, double pow);

    /*
     * FrFunction_
     */

    FrFunction_::FrFunction_() : FrFunctionBase() {};



    /*
     * FrCompositeFunction
     */

    FrCompositeFunction::FrCompositeFunction(frydom::FrFunctionBase& function1, const frydom::FrFunctionBase& function2,
                                             FrCompositeFunction::OPERATOR op) :
                                                FrFunctionBase(),
                                                m_operator(op) {
        m_f1 = function1.Clone();
        m_f2 = function2.Clone();

    }

    FrCompositeFunction::FrCompositeFunction(const FrCompositeFunction &other) : FrFunctionBase() {
        m_f1 = other.m_f1;
        m_f2 = other.m_f2;
        m_operator = other.m_operator;
    }

    FrCompositeFunction* FrCompositeFunction::Clone() const {
        return new FrCompositeFunction(*this);
    }

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

    FrNegateFunction::FrNegateFunction(FrFunctionBase& functionToNegate) : FrFunctionBase() {
        m_functionToNegate = functionToNegate.Clone();
    }

    FrNegateFunction::FrNegateFunction(const frydom::FrNegateFunction &other) : FrFunctionBase() {
        m_functionToNegate = other.m_functionToNegate;
    }

    FrNegateFunction* FrNegateFunction::Clone() const {
        return new FrNegateFunction(*this);
    }



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

    FrMultiplyByScalarFunction::FrMultiplyByScalarFunction(const FrFunctionBase& functionToScale, double alpha) :
        FrFunctionBase() , m_alpha(alpha) {
        m_functionToScale = functionToScale.Clone();
    }

    FrMultiplyByScalarFunction::FrMultiplyByScalarFunction(const FrMultiplyByScalarFunction &other) {
        m_alpha = other.m_alpha;
        m_functionToScale = other.m_functionToScale;
    }

    FrMultiplyByScalarFunction* FrMultiplyByScalarFunction::Clone() const {
        return new FrMultiplyByScalarFunction(*this);
    }

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

    FrInverseFunction::FrInverseFunction(const FrFunctionBase& functionToInverse, double alpha) :
        FrFunctionBase(), m_alpha(alpha) {
        m_functionToInverse = functionToInverse.Clone();
    }

    FrInverseFunction::FrInverseFunction(const frydom::FrInverseFunction &other) {
        m_alpha = other.m_alpha;
        m_functionToInverse = other.m_functionToInverse;
//        m_xOffset = m_xOffset;
    }

    FrInverseFunction* FrInverseFunction::Clone() const {
        return new FrInverseFunction(*this);
    }

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

    /*
     * FrAddScalarToFunction
     */

    FrAddScalarToFunction::FrAddScalarToFunction(const FrFunctionBase& functionToInverse, double alpha) :
        FrFunctionBase(), m_alpha(alpha) {
        m_functionToAddScalar = functionToInverse.Clone();
    }

    FrAddScalarToFunction::FrAddScalarToFunction(const FrAddScalarToFunction &other) {
        m_alpha = other.m_alpha;
        m_functionToAddScalar = other.m_functionToAddScalar;
//        m_xOffset = m_xOffset;
    }

    FrAddScalarToFunction* FrAddScalarToFunction::Clone() const {
        return new FrAddScalarToFunction(*this);
    }

    void FrAddScalarToFunction::Eval(double x) const {
        if (IsEval(x)) return;
        c_x = x;

        c_y = m_alpha + m_functionToAddScalar->Get_y(x);
        c_y_dx = m_functionToAddScalar->Get_y_dx(x);
        c_y_dxdx = m_functionToAddScalar->Get_y_dxdx(x);
    }




}  // end namespace frydom
