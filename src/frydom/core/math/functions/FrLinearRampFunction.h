//
// Created by frongere on 06/02/19.
//

#ifndef FRYDOM_FRRAMP_H
#define FRYDOM_FRRAMP_H

#include "FrFunction.h"


namespace frydom {


    /// Class for representing a ramp
    ///
    ///             --------
    ///            /
    ///           /
    ///          /
    /// --------
    ///
    ///
    class FrRampFunction_ : public FrFunction_ {

    private:

        double m_slope = 1.;
        double m_intercept = 0.;

        double m_x0 = 0.; // TODO : plutot reposer sur l'offset de la fonction de base !
        double m_x1 = 1.;

    public:

        FrRampFunction_();

        FrRampFunction_(const FrRampFunction_& other);

        FrRampFunction_* Clone() const;

        void SetY0(double intercept);

        double GetY0() const;

        void SetSlope(double slope);

        double GetSlope() const;

        void SetInterceptAndSlope(double intercept, double slope);

        void SetXWindow(double x0, double x1);

        void SetByTwoPoints(double x0, double y0, double x1, double y1);

        void Initialize() override;

    protected:
        void Eval(double x) const override;



    };


}  // end namespace frydom

#endif //FRYDOM_FRRAMP_H
