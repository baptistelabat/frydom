//
// Created by frongere on 06/02/19.
//

#ifndef FRYDOM_FRRAMP_H
#define FRYDOM_FRRAMP_H

#include "FrFunction.h"



// Forward declaration
namespace chrono {
    class ChFunction_Ramp;
}

namespace frydom {


    class FrRamp_ : public FrFunction_ {

    private:

        double m_slope = 1.;
        double m_intercept = 0.;


        bool m_isWindowed = false;
        double m_xmin = 0.;
        double m_xmax = 0.;

        double c_ymin = 0.;
        double c_ymax = 1.;

    public:

        FrRamp_();

        void SetY0(double intercept);

        double GetY0() const;

        void SetSlope(double slope);

        double GetSlope() const;

        void Set(double intercept, double slope);

        void SetIsWindowed(bool limit);

        bool GetIsLimited() const;

        void SetXWindow(double xmin, double xmax);

        void SetByTwoPoints(double xmin, double ymin, double xmax, double ymax, bool isWindowed);

//        double Get_y(double x) const override;
//
//        double Get_y_dx(double x) const override;
//
//        double Get_y_dxdx(double x) const override;

        void Initialize() override;

    protected:
        void Eval(double x) const override;



    };


}  // end namespace frydom

#endif //FRYDOM_FRRAMP_H
