//
// Created by frongere on 12/10/18.
//

#ifndef FRYDOM_FRFUNCTION_H
#define FRYDOM_FRFUNCTION_H


#include "chrono/motion_functions/ChFunction.h"

#include "frydom/core/FrObject.h"


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






























    /// REFACTORING ------------->>>>>>>>>>>>>>>>>>

    class FrFunction_ : public FrObject {
    protected:
        double m_functionValue = 0.;
        double c_time = 0.;
    public:

        double GetFunctionValue() const;

        void Update(double time);
    };

    class FrRamp_ : public FrFunction_ {

    private:
        bool m_active = false;

        bool m_increasing = true;
        double m_t0 = 0.;
        double m_t1 = 20.;

        double m_min = 0.;
        double m_max = 1.;

        double c_a = 0.;
        double c_b = 1.;


    public:

        void SetDuration(double duration);

        void SetIncrease();

        void SetDecrease();

        void SetMinVal(double minVal);

        void SetMaxVal(double maxVal);

        bool IsActive();

        void Activate();

        void Deactivate();

        void Initialize() override;

        void StepFinalize() override;

    };


}  // end namespace frydom

#endif //FRYDOM_FRFUNCTION_H
