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


//    class FrFunction_ : public FrObject {
//    protected:
//        double m_functionValue = 1.;    ///< value of the function to return
//        double c_time = 0.;             ///< cached value of the time, updated using the Update method
//    public:
//
//        /// Get the function value
//        /// \return function value
//        double GetFunctionValue() const;
//
//        /// Update the state of the function, including the cached value of the time. This method needs to  be called
//        /// by the container of this function.
//        /// \param time time of the simulation
//        void Update(double time);
//    };
//
//
//
//
//    class FrRamp_ : public FrFunction_ {
//
//    private:
//        bool m_active = false;      ///< bool checking if the ramp is active
//        bool m_increasing = true;   ///< bool checking if the ramp is increasing (true) of decreasing (false)
//
//        double m_t0 = 0.;           ///< start time of the ramp
//        double m_t1 = 20.;          ///< end time of the ramp
//
//        double m_min = 0.;          ///< start value of the ramp if it's increasing, end otherwise
//        double m_max = 1.;          ///< end value of the ramp if it's increasing, start otherwise
//
//        double c_a = 0.;            ///< cached coefficient of the ramp (slope)
//        double c_b = 1.;            ///< cached coefficient of the ramp (y offset)
//
//    public:
//
//        /// Set the duration of the ramp, meaning m_t1-m_t0
//        /// \param duration duration of the ramp
//        void SetDuration(double duration);
//
//        /// Set the ramp to an increasing one
//        void SetIncrease();
//
//        /// Set the ramp to a decreasing one
//        void SetDecrease();
//
//        /// Set the min time of the ramp, m_t0
//        /// \param minTime min time of the ramp
//        void SetMinTime(double minTime);
//
//        /// Set the max time of the ramp, m_t1
//        /// \param maxTime max time of the ramp
//        void SetMaxTime(double maxTime);
//
//        /// Set the min value of the ramp, m_min
//        /// \param minVal min value of the ramp
//        void SetMinVal(double minVal);
//
//        /// Set the max value of the ramp, m_max
//        /// \param maxVal max value of the ramp
//        void SetMaxVal(double maxVal);
//
//        /// Check is the ramp is active
//        /// \return true if the ramp is active, false otherwise
//        bool IsActive();
//
//        /// Activate the ramp (set active)
//        void Activate();
//
//        /// De-activate the ramp (set inactive)
//        void Deactivate();
//
//        /// Initialize the state of the ramp
//        void Initialize() override;
//
//        /// Method called at the send of a time step. Logging may be used here
//        void StepFinalize() override;
//
//    };






//////////// ----------- >>>>> REFACTORING OF THE REFACTORING :) :)


//    // Forward declaration
//    class FrFunction_;
//
//    namespace internal {
//
//        struct FrFunctionWrapper : public chrono::ChFunction {
//
//            FrFunction_* m_frydomFunction;
//
//            explicit FrFunctionWrapper(FrFunction_* frydomFunction);
//
//            double Get_y(double x) const override;
//
//            double Get_y_dx(double x) const override;
//
//            double Get_y_dxdx(double x) const override;
//
//        };
//
//    }  // end namespace frydom::internal



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
    class FrCompositeFunction;
    class FrNegateFunction;
    class FrMultiplyByScalarFunction;
    class FrInverseFunction;

    class FrBaseFunction : public FrObject {

    private:
        bool m_isActive = true;


    public:

        bool IsActive() const;  // TODO : supprimer cette fonctionnalite, repercuter dans le reste du code...

        void SetActive(bool active);

        virtual double Get_y(double x) const = 0;

        virtual double Get_y_dx(double x) const = 0;

        virtual double Get_y_dxdx(double x) const = 0;

        std::shared_ptr<FrCompositeFunction> operator+(std::shared_ptr<FrBaseFunction> otherFunction);

        std::shared_ptr<FrNegateFunction> operator-();

        std::shared_ptr<FrCompositeFunction> operator-(std::shared_ptr<FrBaseFunction> otherFunction);

        std::shared_ptr<FrCompositeFunction> operator*(std::shared_ptr<FrBaseFunction> otherFunction);

        std::shared_ptr<FrCompositeFunction> operator/(std::shared_ptr<FrBaseFunction> otherFunction);

        std::shared_ptr<FrCompositeFunction> operator<<(std::shared_ptr<FrBaseFunction> otherFunction);

        std::shared_ptr<FrMultiplyByScalarFunction> operator*(double alpha);

        std::shared_ptr<FrMultiplyByScalarFunction> operator/(double alpha);

        // TODO : VOIR SI ON GARDE
        void Initialize() override {}
//        void Update(double time) {}
        void StepFinalize() override {}

    };

    /// Left multiplication of a function by a scalar
    std::shared_ptr<FrMultiplyByScalarFunction> operator*(double alpha, std::shared_ptr<FrBaseFunction> otherFunction);

    std::shared_ptr<FrInverseFunction> operator/(double alpha, std::shared_ptr<FrBaseFunction> otherFunction);



    class FrFunction_ : public FrBaseFunction {

    protected:

        std::shared_ptr<chrono::ChFunction> m_chronoFunction;

    public:
//        FrFunction_();

        double Get_y(double x) const override;

        double Get_y_dx(double x) const override;

        double Get_y_dxdx(double x) const override;


    };



    class FrCompositeFunction : public FrBaseFunction {
        friend class FrBaseFunction;

    protected:
        enum OPERATOR {
            ADD, // +
            SUB, // -
            MUL, // *
            DIV, // /
            COM  // << (composition of functions)
        };

    private:
        FrBaseFunction* m_f1; // Il va certainement falloir stocker la fonction produite quelque part !
        FrBaseFunction* m_f2;
        OPERATOR m_operator;

    public:

        FrCompositeFunction(FrBaseFunction* function1, FrBaseFunction* function2, OPERATOR op);

        double Get_y(double x) const override;

        double Get_y_dx(double x) const override;

        double Get_y_dxdx(double x) const override;

    };




    class FrNegateFunction : public FrBaseFunction {

    private:
        FrBaseFunction* m_functionToNegate;

    public:
        explicit FrNegateFunction(FrBaseFunction* functionToNegate);

        double Get_y(double x) const override;

        double Get_y_dx(double x) const override;

        double Get_y_dxdx(double x) const override;

    };



    class FrMultiplyByScalarFunction : public FrBaseFunction {

    private:
        double m_alpha;
        FrBaseFunction* m_functionToScale;

    public:
        FrMultiplyByScalarFunction(FrBaseFunction* functionToScale, double alpha);

        double Get_y(double x) const override;

        double Get_y_dx(double x) const override;

        double Get_y_dxdx(double x) const override;

    };


    class FrInverseFunction : public FrBaseFunction { // TODO : mettre des gardes fou pour la division par 0...

    private:
        double m_alpha;
        FrBaseFunction* m_functionToInverse;

    public:
        FrInverseFunction(FrBaseFunction* functionToScale, double alpha);

        double Get_y(double x) const override;

        double Get_y_dx(double x) const override;

        double Get_y_dxdx(double x) const override;

    };




}  // end namespace frydom

#endif //FRYDOM_FRFUNCTION_H
