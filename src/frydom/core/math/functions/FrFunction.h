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
    class FrFunction_;

    namespace internal {

        struct FrChronoFunctionWrapper : public chrono::ChFunction {

            FrFunction_* m_frydomFunction;

            explicit FrChronoFunctionWrapper(FrFunction_* frydomFunction);

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

    /*
     * FrFunction_
     */

    class FrFunction_ : public FrObject {

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

        FrFunction_();

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

        FrCompositeFunction operator+(const FrFunction_& otherFunction);

        FrNegateFunction operator-();

        FrCompositeFunction operator-(const FrFunction_& otherFunction);

        FrCompositeFunction operator*(const FrFunction_& otherFunction);

        FrCompositeFunction operator/(const FrFunction_& otherFunction);

        FrCompositeFunction operator<<(const FrFunction_& otherFunction);

        FrMultiplyByScalarFunction operator*(double alpha);

        FrMultiplyByScalarFunction operator/(double alpha);

        void WriteToGnuPlotFile(double xmin, double xmax, double dx, std::string filename = "functionOutput") const;

    protected:

        virtual void Eval(double x) const = 0;

        std::shared_ptr<chrono::ChFunction> GetChronoFunction();

        double Estimate_y_dx(double x) const;

        double Estimate_y_dxdx(double x) const;

        inline bool IsEval(double x) const {
            return c_x == x;
        }

    };

    /// Left multiplication of a function by a scalar
    FrMultiplyByScalarFunction operator*(double alpha, const FrFunction_& otherFunction);

    FrInverseFunction operator/(double alpha, const FrFunction_& otherFunction);



    /*
     * FrCompositeFunction
     */

    class FrCompositeFunction : public FrFunction_ {

    protected:
        friend class FrFunction_;
        enum OPERATOR {
            ADD, // +
            SUB, // -
            MUL, // *
            DIV, // /
            COM  // << (composition of functions)
        };

    private:
        FrFunction_* m_f1; // Il va certainement falloir stocker la fonction produite quelque part !
        FrFunction_* m_f2;
        OPERATOR m_operator;

    public:

        FrCompositeFunction(FrFunction_* function1, FrFunction_* function2, OPERATOR op);

//        double Get_y(double x) const override;
//
//        double Get_y_dx(double x) const override;
//
//        double Get_y_dxdx(double x) const override;



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




    class FrNegateFunction : public FrFunction_ {

    private:
        FrFunction_* m_functionToNegate;

    public:
        explicit FrNegateFunction(FrFunction_* functionToNegate);

//        double Get_y(double x) const override;
//
//        double Get_y_dx(double x) const override;
//
//        double Get_y_dxdx(double x) const override;

    protected:
        void Eval(double x) const override;

    };



    class FrMultiplyByScalarFunction : public FrFunction_ {

    private:
        double m_alpha;
        FrFunction_* m_functionToScale;

    public:
        FrMultiplyByScalarFunction(FrFunction_* functionToScale, double alpha);

//        double Get_y(double x) const override;
//
//        double Get_y_dx(double x) const override;
//
//        double Get_y_dxdx(double x) const override;

    protected:
        void Eval(double x) const override;

    };


    class FrInverseFunction : public FrFunction_ { // TODO : mettre des gardes fou pour la division par 0...

    private:
        double m_alpha;
        FrFunction_* m_functionToInverse;

    public:
        FrInverseFunction(FrFunction_* functionToScale, double alpha);

//        double Get_y(double x) const override;
//
//        double Get_y_dx(double x) const override;
//
//        double Get_y_dxdx(double x) const override;

    protected:
        void Eval(double x) const override;

    };




}  // end namespace frydom

#endif //FRYDOM_FRFUNCTION_H
