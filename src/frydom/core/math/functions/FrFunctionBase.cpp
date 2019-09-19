//
// Created by frongere on 12/10/18.
//

#include "FrFunctionBase.h"

#include "fmt/format.h"



// Defines the dx for numerical differentiation
#define FR_BDF  1e-7


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
//        m_chronoFunction = std::make_shared<internal::FrChronoFunctionWrapper>(this);
    }

    FrFunctionBase::~FrFunctionBase() {
        delete m_function; // TODO : voir le concept de virtual destructor, voir aussi si on a des fuites memoire de ce fait...
    }

    FrFunctionBase::FrFunctionBase(const FrFunctionBase& other) {
        if (other.m_function) {
            m_function = other.m_function->Clone();
        } else {
            m_function = nullptr;
        }

        // Copying the cache
        c_x = other.c_x;
        c_y = other.c_y;
        c_y_dx = other.c_y_dx;
        c_y_dxdx = other.c_y_dxdx;
    }

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

    void FrFunctionBase::Initialize() {}

    double FrFunctionBase::operator()(double x) const {
        return Get_y(x);
    }

//    std::shared_ptr<chrono::ChFunction> FrFunctionBase::GetChronoFunction() {
//        return m_chronoFunction;
//    }

    void FrFunctionBase::WriteToGnuPlotFile(double xmin, double xmax, double dx, std::string filename) const {

        fmt::MemoryWriter mw;

        // Building Gnuplot dat file as a single string
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

        // Writing gnuplot file along with gnuplot script syntax
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
        return (Get_y(x + FR_BDF) - Get_y(x)) / FR_BDF;
    }

    double FrFunctionBase::Estimate_y_dxdx(double x) const {
        return (Get_y_dx(x + FR_BDF) - Get_y_dx(x)) / FR_BDF;
    }


    /*
     * Operators
     */


    FrUnaryOpFunction FrFunctionBase::operator-() {
        return FrUnaryOpFunction(*this, true);
    }

    FrUnaryOpFunction FrFunctionBase::operator+() {
        return FrUnaryOpFunction(*this, false);
    }



    FrAddFunction FrFunctionBase::operator+(const FrFunctionBase& other) {
        return FrAddFunction(*this, other);
    }

    FrSubFunction FrFunctionBase::operator-(const FrFunctionBase& other) {
        return FrSubFunction(*this, other);
    }

    FrMulFunction FrFunctionBase::operator*(const FrFunctionBase& other) {
        return FrMulFunction(*this, other);
    }

    FrDivFunction FrFunctionBase::operator/(const FrFunctionBase& other) {
        return FrDivFunction(*this, other);
    }

    FrCompFunction FrFunctionBase::operator<<(const FrFunctionBase& other) {
        return FrCompFunction(*this, other);
    }




    FrMulFunction FrFunctionBase::operator*(double alpha) {
        return FrMulFunction(*this, FrConstantFunction(alpha));
    }

    FrDivFunction FrFunctionBase::operator/(double alpha) {
        return FrDivFunction(*this, FrConstantFunction(alpha));
    }

    FrAddFunction FrFunctionBase::operator+(double alpha) {
        return FrAddFunction(*this, FrConstantFunction(alpha));
    }

    FrSubFunction FrFunctionBase::operator-(double alpha) {
        return FrSubFunction(*this, FrConstantFunction(alpha));
    }

    void FrFunctionBase::operator+=(const FrFunctionBase& other) {
        m_function = FrAddFunction(*m_function, other).Clone();
    }

    void FrFunctionBase::operator-=(const FrFunctionBase& other) {
        m_function = FrSubFunction(*m_function, other).Clone();
    }

    void FrFunctionBase::operator*=(const FrFunctionBase& other) {
        m_function = FrMulFunction(*m_function, other).Clone();
    }

    void FrFunctionBase::operator/=(const FrFunctionBase& other) {
        m_function = FrDivFunction(*m_function, other).Clone();
    }

    void FrFunctionBase::operator+=(double alpha) {
        m_function = FrAddFunction(*m_function, FrConstantFunction(alpha)).Clone();
    }

    void FrFunctionBase::operator-=(double alpha) {
        m_function = FrSubFunction(*m_function, FrConstantFunction(alpha)).Clone();
    }

    void FrFunctionBase::operator*=(double alpha) {
        m_function = FrMulFunction(*m_function, FrConstantFunction(alpha)).Clone();
    }

    void FrFunctionBase::operator/=(double alpha) {
        m_function = FrDivFunction(*m_function, FrConstantFunction(alpha)).Clone();
    }

    FrAddFunction operator+(double alpha, const FrFunctionBase& function) {
        return FrAddFunction(FrConstantFunction(alpha), function);
    }

    FrSubFunction operator-(double alpha, const FrFunctionBase& function) {
        return FrSubFunction(FrConstantFunction(alpha), function);
    }

    FrMulFunction operator*(double alpha, const FrFunctionBase& function) {
        return FrMulFunction(FrConstantFunction(alpha), function);
    }

    FrDivFunction operator/(double alpha, const FrFunctionBase& function) {
        return FrDivFunction(FrConstantFunction(alpha), function);
    }


    namespace internal {

        FrFunctionChronoInterface::FrFunctionChronoInterface(const frydom::FrFunctionBase &frydomFunction) {
            m_function = frydomFunction.Clone();
            m_chronoFunction = std::make_shared<internal::FrChronoFunctionWrapper>(m_function);
        }

        std::shared_ptr<internal::FrChronoFunctionWrapper> FrFunctionChronoInterface::GetChronoFunction() {
            return m_chronoFunction;
        }

    }  // end namespace frydom::internal



    FrVarXFunction::FrVarXFunction() {
        m_function = nullptr;
    };

    FrVarXFunction::FrVarXFunction(std::string varname) : FrVarXFunction() {
        m_varname = varname;
    }

    FrVarXFunction::FrVarXFunction(const FrVarXFunction& other) : FrFunctionBase(other) {}

    FrVarXFunction* FrVarXFunction::Clone() const {
        return new FrVarXFunction(*this);
    }

    std::string FrVarXFunction::GetRepr() const {
        return m_varname;
    }

    void FrVarXFunction::Eval(double x) const {
        c_x = x;
        c_y = x;
        c_y_dx = 1;
        c_y_dxdx = 0.;
    }

    FrVarXFunction new_var() {
        return FrVarXFunction();
    }

    FrVarXFunction new_var(std::string varname) {
        return FrVarXFunction(varname);
    }


    /*
     * FrConstantFunction
     */

    FrConstantFunction::FrConstantFunction(double scalar) {
        m_function = nullptr;
        c_y = scalar;
        c_y_dx = 0.;
        c_y_dxdx = 0.;
    }

    FrConstantFunction::FrConstantFunction(const frydom::FrConstantFunction &other) : FrFunctionBase(other) {}

    FrConstantFunction* FrConstantFunction::Clone() const {
        return new FrConstantFunction(*this);
    }

    void FrConstantFunction::Set(double scalar) {
        c_y = scalar;
    }

    double FrConstantFunction::Get() const {
        return c_y;
    }

    double FrConstantFunction::operator()() const {
        return c_y;
    }

    double& FrConstantFunction::operator()() {
        return c_y;
    }

    std::string FrConstantFunction::GetRepr() const {
        fmt::MemoryWriter mw;
        mw << c_y;
        return mw.str();
    }

    void FrConstantFunction::Eval(double x) const {}


    /*
     * FrUnaryOpFunction
     */

    FrUnaryOpFunction::FrUnaryOpFunction(const FrFunctionBase& function, bool negate) {
        m_function = function.Clone();
        m_negate = negate;
    }

    FrUnaryOpFunction::FrUnaryOpFunction(const FrUnaryOpFunction &other) : FrFunctionBase(other) {
        m_negate = other.m_negate;
    }

    FrUnaryOpFunction* FrUnaryOpFunction::Clone() const {
        return new FrUnaryOpFunction(*this);
    }

    std::string FrUnaryOpFunction::GetRepr() const {
        fmt::MemoryWriter mw;
        if (m_negate) mw << '-';

        mw << m_function->GetRepr();
        return mw.str();
    }

    void FrUnaryOpFunction::Eval(double x) const {
        if (IsEval(x)) return;
        c_x = x;
        if (m_negate) {
            c_y = -m_function->Get_y(x);
            c_y_dx = -m_function->Get_y_dx(x);
            c_y_dxdx = -m_function->Get_y_dxdx(x);
        } else {
            c_y = m_function->Get_y(x);
            c_y_dx = m_function->Get_y_dx(x);
            c_y_dxdx = m_function->Get_y_dxdx(x);
        }
    }



    FrBinaryOpFunction::FrBinaryOpFunction(const FrFunctionBase& f1, const FrFunctionBase& f2) : FrFunctionBase() {
        m_function = f1.Clone();
        m_rightFunction = f2.Clone();
    }

    FrBinaryOpFunction::FrBinaryOpFunction(const FrBinaryOpFunction& other) : FrFunctionBase(other) {
        m_rightFunction = other.m_rightFunction->Clone(); // FIXME : tester sans, voir si on passe ici... c'est a priori fait pas le constructeur par copie de FrFunctionBase....
    }


    FrAddFunction::FrAddFunction(const FrFunctionBase &f1, const FrFunctionBase &f2) : FrBinaryOpFunction(f1, f2) {}

    FrAddFunction::FrAddFunction(const FrAddFunction &other) : FrBinaryOpFunction(other) {}

    FrAddFunction *FrAddFunction::Clone() const {
        return new FrAddFunction(*this);
    }

    std::string FrAddFunction::GetRepr() const {
        fmt::MemoryWriter mw;
        mw << m_function->GetRepr() << " + " << m_rightFunction->GetRepr();
        return mw.str();
    }

    void FrAddFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        c_y = m_function->Get_y(x) + m_rightFunction->Get_y(x);
        c_y_dx = m_function->Get_y_dx(x) + m_rightFunction->Get_y_dx(x);
        c_y_dxdx = m_function->Get_y_dxdx(x) + m_rightFunction->Get_y_dxdx(x);

    }

    FrSubFunction::FrSubFunction(const FrFunctionBase &f1, const FrFunctionBase &f2) : FrBinaryOpFunction(f1, f2) {}

    FrSubFunction::FrSubFunction(const FrSubFunction &other) : FrBinaryOpFunction(other) {}

    FrSubFunction *FrSubFunction::Clone() const {
        return new FrSubFunction(*this);
    }

    std::string FrSubFunction::GetRepr() const {
        fmt::MemoryWriter mw;
        mw << m_function->GetRepr() << " - " << m_rightFunction->GetRepr();
        return mw.str();
    }

    void FrSubFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        c_y = m_function->Get_y(c_x) - m_rightFunction->Get_y(c_x);
        c_y_dx   = m_function->Get_y_dx(c_x) - m_rightFunction->Get_y_dx(c_x);
        c_y_dxdx = m_function->Get_y_dxdx(c_x) - m_rightFunction->Get_y_dxdx(c_x);
    }

    FrMulFunction::FrMulFunction(const FrFunctionBase &f1, const FrFunctionBase &f2) : FrBinaryOpFunction(f1, f2) {}

    FrMulFunction::FrMulFunction(const FrMulFunction &other) : FrBinaryOpFunction(other) {}

    FrMulFunction *FrMulFunction::Clone() const {
        return new FrMulFunction(*this);
    }

    std::string FrMulFunction::GetRepr() const {
        fmt::MemoryWriter mw;
        mw << m_function->GetRepr() << " * " << m_rightFunction->GetRepr();
        return mw.str();
    }

    void FrMulFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double u, u_dx, u_dxdx, v, v_dx, v_dxdx;
        EvalFunctions(u, u_dx, u_dxdx, v, v_dx, v_dxdx);

        c_y      = u * v;
        c_y_dx   = u_dx * v + v_dx * u;
        c_y_dxdx = u_dxdx*v + 2.*u_dx*v_dx + v_dxdx*u;
    }

    FrDivFunction::FrDivFunction(const FrFunctionBase &f1, const FrFunctionBase &f2) : FrBinaryOpFunction(f1, f2) {}

    FrDivFunction::FrDivFunction(const FrDivFunction &other) : FrBinaryOpFunction(other) {}

    FrDivFunction *FrDivFunction::Clone() const {
        return new FrDivFunction(*this);
    }

    std::string FrDivFunction::GetRepr() const {
        fmt::MemoryWriter mw;
        mw << m_function->GetRepr() << " / " << m_rightFunction->GetRepr();
        return mw.str();
    }

    void FrDivFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double u, u_dx, u_dxdx, v, v_dx, v_dxdx;
        EvalFunctions(u, u_dx, u_dxdx, v, v_dx, v_dxdx);

        c_y = u / v;
        c_y_dx = (u_dx * v - v_dx * u) / (v*v);
        c_y_dxdx = (u_dxdx - 2. * c_y_dx * v_dx - c_y * v_dxdx) / v;
    }

    FrCompFunction::FrCompFunction(const FrFunctionBase &f1, const FrFunctionBase &f2) : FrBinaryOpFunction(f1, f2) {}

    FrCompFunction::FrCompFunction(const FrCompFunction &other) : FrBinaryOpFunction(other) {}

    FrCompFunction *FrCompFunction::Clone() const {
        return new FrCompFunction(*this);
    }

    std::string FrCompFunction::GetRepr() const {
        fmt::MemoryWriter mw;
        mw << m_function->GetRepr() << " o(" << m_rightFunction->GetRepr()<<')';
        return mw.str();
    }

    void FrCompFunction::Eval(double x) const {
        if (IsEval(x)) return;

        c_x = x;
        double v    = m_rightFunction->Get_y(c_x);
        double v_dx = m_rightFunction->Get_y_dx(c_x);
        double v_dxdx = m_rightFunction->Get_y_dxdx(c_x);

        c_y = m_function->Get_y(v);
        c_y_dx = v_dx * m_function->Get_y_dx(v);
        c_y_dxdx = v_dxdx * m_function->Get_y_dx(v) + v_dx*v_dx * m_function->Get_y_dxdx(v);
    }


}  // end namespace frydom
