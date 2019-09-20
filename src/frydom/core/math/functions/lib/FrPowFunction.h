//
// Created by frongere on 11/02/19.
//

#ifndef FRYDOM_FRPOWFUNCTION_H
#define FRYDOM_FRPOWFUNCTION_H

#include "frydom/core/math/functions/FrFunctionBase.h"

namespace frydom {

    class FrPowFunction : public FrFunctionBase {

    private:
        double m_power = 1.;

    public:
        FrPowFunction(const FrFunctionBase& function, double power);
        FrPowFunction(const FrPowFunction& other);
        FrPowFunction* Clone() const override;

        void Set(double power);

        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "PowFunction"; }

    protected:
        void Eval(double x) const;

    };

    FrPowFunction pow(const FrFunctionBase& function, double power);

}  // end namespace frydom

#endif //FRYDOM_FRPOWFUNCTION_H
