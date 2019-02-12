//
// Created by frongere on 11/02/19.
//

#ifndef FRYDOM_FRLINEARFUNCTION_H
#define FRYDOM_FRLINEARFUNCTION_H


#include "frydom/core/math/functions/FrFunctionBase.h"


namespace frydom {

    class FrLinearFunction : public FrFunction_ {

    private:
        double m_intercept = 0.;  //
        double m_slope = 1.;

    public:
        FrLinearFunction(double intercept, double slope);
        FrLinearFunction(const FrLinearFunction& other);
        FrLinearFunction* Clone() const override;

        void SetByTwoPoints(double x0, double y0, double x1, double y1);
        void SetSlope(double slope);
        double GetSlope() const;
        void SetIntercept(double intercept);
        double GetIntercept() const;
        void Set(double intercept, double slope);

        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEARFUNCTION_H
