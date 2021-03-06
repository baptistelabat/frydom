//
// Created by frongere on 12/02/19.
//

#ifndef FRYDOM_FRMISCFUNCTIONS_H
#define FRYDOM_FRMISCFUNCTIONS_H

#include "frydom/core/math/functions/FrFunctionBase.h"

namespace frydom {

  class FrSqrtFunction : public FrFunctionBase {

   public:
    explicit FrSqrtFunction(double alpha);

    FrSqrtFunction(const FrFunctionBase &function);

    FrSqrtFunction(const FrSqrtFunction &other);

    FrSqrtFunction *Clone() const override;

    std::string GetRepr() const override;

   protected:
    void Eval(double x) const override;

  };

  FrSqrtFunction sqrt(const FrFunctionBase &function);


  class FrAbsFunction : public FrFunctionBase {

   public:
    explicit FrAbsFunction(double alpha);

    FrAbsFunction(const FrFunctionBase &function);

    FrAbsFunction(const FrAbsFunction &other);

    FrAbsFunction *Clone() const override;

    std::string GetRepr() const override;

   protected:
    void Eval(double x) const override;

  };

  FrAbsFunction abs(const FrFunctionBase &function);


}  // end namespace frydom



#endif //FRYDOM_FRMISCFUNCTIONS_H
