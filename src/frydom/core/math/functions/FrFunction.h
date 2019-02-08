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





    /*
     * TODO : voir pour faire :
     *
     * sin
     * tan
     * exp
     * log
     * ch
     * sh
     * th -> utilisation particuliere en tant que rampe
     * polynomial (arbitrary order)
     * spline
     * depuis fichier
     *
     *
     */

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
    class FrCompositeFunction;
    class FrNegateFunction;
    class FrMultiplyByScalarFunction;
    class FrInverseFunction;
    class FrAddScalarToFunction;

    /*
     * FrFunction_
     */

    class FrFunctionBase : public FrObject {

    private:

        std::shared_ptr<internal::FrChronoFunctionWrapper> m_chronoFunction;

        bool m_isActive = true; // TODO : retirer ?

    protected:

        double m_xOffset = 0.;

        // Cache
        mutable double c_x;
        mutable double c_y;
        mutable double c_y_dx;
        mutable double c_y_dxdx;

    public:

        FrFunctionBase();

        /// Virtual copy constructor
        virtual FrFunctionBase* Clone() const = 0;

        bool IsActive() const;  // TODO : supprimer cette fonctionnalite, repercuter dans le reste du code...
        void SetActive(bool active);

        void SetXOffset(double xOffset);
        double GetXOffset() const;

        double Get_y(double x) const;
        double Get_y_dx(double x) const;
        double Get_y_dxdx(double x) const;

        void Initialize() override {}
        void StepFinalize() override;

        double operator()(double x) const;

        // TODO : ajouter operateurs pour ajout de scalaire...

        void WriteToGnuPlotFile(double xmin, double xmax, double dx, std::string filename = "functionOutput") const;

        /*
         * Operators
         */

        /// Add two functions
        FrCompositeFunction operator+(FrFunctionBase& other);

        /// Substract two functions
        FrCompositeFunction operator-(FrFunctionBase& other);

        /// Multiplpy two functions
        FrCompositeFunction operator*(FrFunctionBase& other);

        /// Divide two functions
        FrCompositeFunction operator/(FrFunctionBase& other);

        /// Compose two functions -> this(other(x))
        FrCompositeFunction operator<<(FrFunctionBase& other);

        /// Negate a function
        FrNegateFunction operator-();

        /// Right multiply a function by a scalar
        FrMultiplyByScalarFunction operator*(double alpha);

        /// Add a scalar to the function to the right
        FrAddScalarToFunction operator+(double alpha);

        /// Substract a scalar to the function to the right
        FrAddScalarToFunction operator-(double alpha);





    protected:

        virtual void Eval(double x) const = 0;

        std::shared_ptr<chrono::ChFunction> GetChronoFunction();

        double Estimate_y_dx(double x) const;

        double Estimate_y_dxdx(double x) const;

        inline bool IsEval(double x) const {
            return c_x == x;
        }

    };

    /// Left multiply a function by a scalar
    FrMultiplyByScalarFunction operator*(double alpha, FrFunctionBase& functionToScale);

    /// Add a scalar to the function to the left
    FrAddScalarToFunction operator+(double alpha, FrFunctionBase& functionToAddScalar);
//    FrAddScalarToFunction operator+(double&& alpha, const FrFunctionBase& functionToAddScalar);

    /// Inverse a function and multiply by a scalar
    FrInverseFunction operator/(double alpha, FrFunctionBase& functionToInverse);



    /*
     * FrFunction_
     */

    class FrFunction_ : public FrFunctionBase {

    public:
        FrFunction_();

    };



    /*
     * FrCompositeFunction
     */

    class FrCompositeFunction : public FrFunctionBase {

    protected:
        friend class FrFunctionBase;
        enum OPERATOR {
            ADD, // +
            SUB, // -
            MUL, // *
            DIV, // /
            COM  // << (composition of functions)
        };

    private:
        FrFunctionBase* m_f1; // Il va certainement falloir stocker la fonction produite quelque part !
        FrFunctionBase* m_f2;
        OPERATOR m_operator;

    public:

        FrCompositeFunction(FrFunctionBase& function1, FrFunctionBase& function2, OPERATOR op);

        FrCompositeFunction(const FrCompositeFunction& other);

        FrCompositeFunction* Clone() const;


    private:

        void Eval(double x) const override;

        void EvalADD() const;
        void EvalSUB() const;
        void EvalMUL() const;
        void EvalDIV() const;
        void EvalCOM() const;

        inline void EvalFunctions(double& u, double& u_dx, double& u_dxdx, double& v, double& v_dx, double& v_dxdx) const {
            u = m_f1->Get_y(c_x);
            v = m_f2->Get_y(c_x);

            u_dx = m_f1->Get_y_dx(c_x);
            v_dx = m_f2->Get_y_dx(c_x);

            u_dxdx = m_f1->Get_y_dxdx(c_x);
            v_dxdx = m_f2->Get_y_dxdx(c_x);
        }

    };




    class FrNegateFunction : public FrFunctionBase {

    private:
        FrFunctionBase* m_functionToNegate;

    public:

        explicit FrNegateFunction(FrFunctionBase& functionToNegate);

        FrNegateFunction(const FrNegateFunction& other);

        FrNegateFunction* Clone() const;



    protected:
        void Eval(double x) const override;

    };



    class FrMultiplyByScalarFunction : public FrFunctionBase {

    private:
        double m_alpha;
        FrFunctionBase* m_functionToScale;

    public:
        FrMultiplyByScalarFunction(FrFunctionBase& functionToScale, double alpha);

        FrMultiplyByScalarFunction(const FrMultiplyByScalarFunction& other);

        FrMultiplyByScalarFunction* Clone() const;

    protected:
        void Eval(double x) const override;

    };


    class FrInverseFunction : public FrFunctionBase { // TODO : mettre des gardes fou pour la division par 0...

    private:
        double m_alpha;
        FrFunctionBase* m_functionToInverse;

    public:
        FrInverseFunction(FrFunctionBase& m_functionToInverse, double alpha);

        FrInverseFunction(const FrInverseFunction& other);

        FrInverseFunction* Clone() const;

    protected:
        void Eval(double x) const override;

    };


    class FrAddScalarToFunction : public FrFunctionBase {

    private:
        double m_alpha;
        FrFunctionBase* m_functionToAddScalar;

    public:
        FrAddScalarToFunction(FrFunctionBase& functionToAddScalar, double alpha);

        FrAddScalarToFunction(const FrAddScalarToFunction& other);

        FrAddScalarToFunction* Clone() const;

    protected:
        void Eval(double x) const override;

    };


}  // end namespace frydom

#endif //FRYDOM_FRFUNCTION_H
