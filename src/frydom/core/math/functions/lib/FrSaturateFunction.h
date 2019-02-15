//
// Created by frongere on 11/02/19.
//

#ifndef FRYDOM_FRSATURATEFUNCTION_H
#define FRYDOM_FRSATURATEFUNCTION_H

#include "frydom/core/math/functions/FrFunctionBase.h"


namespace frydom {

    class FrSaturateFunction : public FrFunctionBase {

    private:

        FrFunctionBase* m_function;
        double m_ymin = INFINITY;
        double m_ymax = -INFINITY;

    public:
        explicit FrSaturateFunction(const FrFunctionBase& function);
        FrSaturateFunction(const FrSaturateFunction& other);
        FrSaturateFunction* Clone() const override;

        void SetYMin(double ymin);
        void SetYMax(double ymax);

        std::string GetRepr() const override;

    protected:
        void Eval(double x) const;


    };

    FrSaturateFunction saturate_high(const FrFunctionBase& function, double ymax);
    FrSaturateFunction saturate_low(const FrFunctionBase& function, double ymin);
    FrSaturateFunction saturate_both(const FrFunctionBase& function, double ymin, double ymax);

}  // end namesapce frydom

#endif //FRYDOM_FRSATURATEFUNCTION_H
