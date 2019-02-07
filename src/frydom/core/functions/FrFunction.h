// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRFUNCTION_H
#define FRYDOM_FRFUNCTION_H


#include "chrono/motion_functions/ChFunction.h"

#include "frydom/core/common/FrObject.h"


namespace frydom {


    // TODO : continuer le dev...

    /**
     * \class FrFunction
     * \brief Class for dealing with real functions (ramp, etc.).
     */
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

    /**
     * \class FrFunction_
     * \brief Class for dealing with real functions (ramp, etc.).
     */
    class FrFunction_ : public FrObject {
    protected:
        double m_functionValue = 1.;    ///< value of the function to return
        double c_time = 0.;             ///< cached value of the time, updated using the Update method
    public:

        /// Get the function value
        /// \return function value
        double GetFunctionValue() const;

        /// Update the state of the function, including the cached value of the time. This method needs to  be called
        /// by the container of this function.
        /// \param time time of the simulation
        void Update(double time);
    };

    /**
     * \class FrRamp_
     * \brief Class for dealing with the ramp function.
     */
    class FrRamp_ : public FrFunction_ {

    private:
        bool m_active = false;      ///< bool checking if the ramp is active
        bool m_increasing = true;   ///< bool checking if the ramp is increasing (true) of decreasing (false)

        double m_t0 = 0.;           ///< start time of the ramp
        double m_t1 = 20.;          ///< end time of the ramp

        double m_min = 0.;          ///< start value of the ramp if it's increasing, end otherwise
        double m_max = 1.;          ///< end value of the ramp if it's increasing, start otherwise

        double c_a = 0.;            ///< cached coefficient of the ramp (slope)
        double c_b = 1.;            ///< cached coefficient of the ramp (y offset)

    public:

        /// Set the duration of the ramp, meaning m_t1-m_t0
        /// \param duration duration of the ramp
        void SetDuration(double duration);

        /// Set the ramp to an increasing one
        void SetIncrease();

        /// Set the ramp to a decreasing one
        void SetDecrease();

        /// Set the min time of the ramp, m_t0
        /// \param minTime min time of the ramp
        void SetMinTime(double minTime);

        /// Set the max time of the ramp, m_t1
        /// \param maxTime max time of the ramp
        void SetMaxTime(double maxTime);

        /// Set the min value of the ramp, m_min
        /// \param minVal min value of the ramp
        void SetMinVal(double minVal);

        /// Set the max value of the ramp, m_max
        /// \param maxVal max value of the ramp
        void SetMaxVal(double maxVal);

        /// Check is the ramp is active
        /// \return true if the ramp is active, false otherwise
        bool IsActive();

        /// Activate the ramp (set active)
        void Activate();

        /// De-activate the ramp (set inactive)
        void Deactivate();

        /// Initialize the state of the ramp
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

    };


}  // end namespace frydom

#endif //FRYDOM_FRFUNCTION_H
