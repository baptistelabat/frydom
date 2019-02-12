//
// Created by frongere on 12/10/18.
//

#ifndef FRYDOM_FRFUNCTION_H
#define FRYDOM_FRFUNCTION_H


#include "chrono/motion_functions/ChFunction.h"

#include "frydom/core/common/FrObject.h"


namespace frydom {


    // TODO : continuer le dev...

    class FrFunction : public chrono::ChFunction {

//    protected:
//
//        std::shared_ptr<chrono::ChFunction> m_chronoFunction;

        double m_C = 0.;

        double Tramp = 50;

        double c_a;

    public:


        FrFunction(double C) : chrono::ChFunction(), m_C(C) {

            double eps = 1e-3;
            c_a = 0.5 * log((1-eps) / eps);

        }


        double Get_y(double x) const override {

//            double val;
//            if (x <= Tramp) {
//                val = 0.5 * m_C * (1. + tanh(2.* c_a * x - c_a * Tramp));
//            } else {
//                val = m_C;
//            }
//            return val;

            double val;
            if (x <= Tramp) {
                val = m_C * x / Tramp;
            } else {
                val = m_C;
            }
            return val;


        }

        FrFunction* Clone() const {
            return new FrFunction(*this);
        }


    };













    // REFACTORING ------------->>>>>>>>>>>>>>>>>>


    // Forward declaration
    class FrFunctionBase;

    namespace internal {

        struct FrChronoFunctionWrapper : public chrono::ChFunction {

            FrFunctionBase* m_frydomFunction;

            explicit FrChronoFunctionWrapper(FrFunctionBase* frydomFunction);

            FrChronoFunctionWrapper* Clone() const override;

            double Get_y(double x) const override;

            double Get_y_dx(double x) const override;

            double Get_y_dxdx(double x) const override;

        };

    }  // end namespace frydom::internal

    // Forward declaration
    class FrUnarySignFunction;
    class FrAddFunction;
    class FrSubFunction;
    class FrMulFunction;
    class FrdivFunction;
    class FrCompFunction;


    /*
     * FrFunctionBase
     */

    class FrFunctionBase : public FrObject {

    private:

//        std::shared_ptr<internal::FrChronoFunctionWrapper> m_chronoFunction;  // TODO : deleguer cet attribut a du chrono interface qu'on placera en interne ?

        bool m_isActive = true; // TODO : retirer ?

    protected:

        FrFunctionBase* m_function; /// The function on which we apply the current function

        double m_xOffset = 0.;  // TODO : supprimer et faire plutot une fonciton retard qui agit sur x...

        // Cache
        mutable double c_x = INFINITY;
        mutable double c_y;
        mutable double c_y_dx;
        mutable double c_y_dxdx;

    public:

        FrFunctionBase();

        ~FrFunctionBase();


        /// Constructor specifying the function on which we apply the current function
        FrFunctionBase(const FrFunctionBase& other);

        /// Virtual copy constructor
        virtual FrFunctionBase* Clone() const = 0;

        bool IsActive() const;  // TODO : supprimer cette fonctionnalite, repercuter dans le reste du code...
        void SetActive(bool active);

        void SetXOffset(double xOffset);
        double GetXOffset() const;

        double Get_y(double x) const;
        double Get_y_dx(double x) const;
        double Get_y_dxdx(double x) const;

        void Initialize() override;
        void StepFinalize() override;

        double operator()(double x) const;

        // TODO : ajouter operateurs pour ajout de scalaire...

        virtual std::string GetRepr() const = 0;

        void WriteToGnuPlotFile(double xmin, double xmax, double dx, std::string filename = "functionOutput") const;

        /*
         * Operators
         */

        /// Negate a function
        FrUnarySignFunction operator-();
        FrUnarySignFunction operator+();


        /// Add two functions
        FrAddFunction operator+(const FrFunctionBase& other);

        /// Substract two functions
        FrSubFunction operator-(const FrFunctionBase& other);

        /// Multiplpy two functions
        FrMulFunction operator*(const FrFunctionBase& other);

        /// Divide two functions
        FrdivFunction operator/(const FrFunctionBase& other);

        /// Compose two functions -> this(other(x))
        FrCompFunction operator<<(const FrFunctionBase& other);



        /// Right multiply a function by a scalar
        FrMulFunction operator*(double alpha);

        /// Right divide a function by a scalar
        FrdivFunction operator/(double alpha);

        /// Add a scalar to the function to the right
        FrAddFunction operator+(double alpha);

        /// Substract a scalar to the function to the right
        FrSubFunction operator-(double alpha);

        /// Add an other function to this function
        void operator+=(const FrFunctionBase& other);

        /// Substract an other function to this function
        void operator-=(const FrFunctionBase& other);

        /// Multiply this function by another function
        void operator*=(const FrFunctionBase& other);

        /// Divide this function by another function
        void operator/=(const FrFunctionBase& other);


        /// Add an other function to this function
        void operator+=(double alpha);

        /// Substract an other function to this function
        void operator-=(double alpha);

        /// Multiply this function by another function
        void operator*=(double alpha);

        /// Divide this function by another function
        void operator/=(double alpha);


    protected:

        virtual void Eval(double x) const = 0;

//        std::shared_ptr<chrono::ChFunction> GetChronoFunction();

        double Estimate_y_dx(double x) const;

        double Estimate_y_dxdx(double x) const;

        inline bool IsEval(double x) const {
            return c_x == x;
        }

    };

    /// Add a scalar to the function to the left
    FrAddFunction operator+(double alpha, const FrFunctionBase& function);

    /// Add a scalar to the function to the left
    FrSubFunction operator-(double alpha, const FrFunctionBase& function);

    /// Left multiply a function by a scalar
    FrMulFunction operator*(double alpha, const FrFunctionBase& function);

    /// Inverse a function and multiply by a scalar
    FrdivFunction operator/(double alpha, const FrFunctionBase& function);


    namespace internal {

        class FrFunctionChronoInterface : public FrFunctionBase {

        private:
            std::shared_ptr<internal::FrChronoFunctionWrapper> m_chronoFunction;

        public:
            explicit FrFunctionChronoInterface(const FrFunctionBase& frydomFunction);

            std::shared_ptr<internal::FrChronoFunctionWrapper> GetChronoFunction();

        };

    }


    class FrVarXFunction : public FrFunctionBase {

    private:
        std::string m_varname = "x";

    public:
        FrVarXFunction();
        FrVarXFunction(std::string varname);
        FrVarXFunction(const FrVarXFunction& other);
        FrVarXFunction* Clone() const override;
        std::string GetRepr() const override;

    protected:
        void Eval(double x) const;

    };

    FrVarXFunction new_var();
    FrVarXFunction new_var(std::string varname);



    /*
     * FrFunction_
     */

//    class FrFunction_ : public FrFunctionBase {
//
////    private:
////        FrFunctionBase* m_function;
//
//    public:
//        FrFunction_();
//        explicit FrFunction_(const FrFunctionBase& function);
//        ~FrFunction_();
//
//    };


    class FrConstantFunction : public FrFunctionBase {

    public:
        explicit FrConstantFunction(double scalar);
        FrConstantFunction(const FrConstantFunction& other);
        FrConstantFunction* Clone() const;

        void Set(double scalar);
        double Get() const;
        double operator()() const;
        double& operator()();
        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };



    /*
     * Results of unary operators
     */

    class FrUnarySignFunction : public FrFunctionBase {

    private:
        bool m_negate = false;

    public:
        FrUnarySignFunction(const FrFunctionBase& function, bool negate);
        FrUnarySignFunction(const FrUnarySignFunction& other);
        FrUnarySignFunction* Clone() const;
        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };


    /*
     * Results of binary operators
     */


    /// Base class for functions that result from a binary operation (abstract)
    class FrBinaryOpFunction : public FrFunctionBase {

    protected:
        FrFunctionBase* m_rightFunction;

    public:
        FrBinaryOpFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrBinaryOpFunction(const FrBinaryOpFunction& other);

    protected:

        inline void EvalFunctions(double& u, double& u_dx, double& u_dxdx, double& v, double& v_dx, double& v_dxdx) const {
            u = m_function->Get_y(c_x);
            v = m_rightFunction->Get_y(c_x);

            u_dx = m_function->Get_y_dx(c_x);
            v_dx = m_rightFunction->Get_y_dx(c_x);

            u_dxdx = m_function->Get_y_dxdx(c_x);
            v_dxdx = m_rightFunction->Get_y_dxdx(c_x);
        }

    };



    /// Class that result from adding two functions
    class FrAddFunction : public FrBinaryOpFunction {

    public:
        FrAddFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrAddFunction(const FrAddFunction& other);
        FrAddFunction* Clone() const;
        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };
    /// Class that result from subtracting two functions
    class FrSubFunction : public FrBinaryOpFunction {

    public:
        FrSubFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrSubFunction(const FrSubFunction& other);
        FrSubFunction* Clone() const;
        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

    /// Class that result from multiplying two functions
    class FrMulFunction : public FrBinaryOpFunction {

    public:
        FrMulFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrMulFunction(const FrMulFunction& other);
        FrMulFunction* Clone() const;
        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

    /// Class that result from dividing two functions
    class FrdivFunction : public FrBinaryOpFunction {

    public:
        FrdivFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrdivFunction(const FrdivFunction& other);
        FrdivFunction* Clone() const;
        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

    /// Class that result from composing two functions
    class FrCompFunction : public FrBinaryOpFunction {

    public:
        FrCompFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrCompFunction(const FrCompFunction& other);
        FrCompFunction* Clone() const;
        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };






}  // end namespace frydom

#endif //FRYDOM_FRFUNCTION_H
