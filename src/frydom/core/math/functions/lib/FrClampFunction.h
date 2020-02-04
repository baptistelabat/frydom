//
// Created by frongere on 12/02/19.
//

#ifndef FRYDOM_FRCLAMPFUNCTION_H
#define FRYDOM_FRCLAMPFUNCTION_H


#include "frydom/core/math/functions/FrFunctionBase.h"

namespace frydom {

  // FIXME : en cas de composition, le clamp n'a pas lieu sur x mais sur le resultat de la
  // fonction embarquee

  class FrClampFunction : public FrFunctionBase {

   protected:
    double m_xClamp;

   public:
    FrClampFunction(const FrFunctionBase &function, double xClamp);

    FrClampFunction(const FrClampFunction &other);
//        FrClampFunction* Clone() const override;

    void Set(double xClamp);

//        std::string GetRepr() const override;

//    protected:
//        void Eval(double x) const override;

  };

  class FrClampAfterFunction : public FrClampFunction {

   public:
    FrClampAfterFunction(const FrFunctionBase &function, double xClamp);

    FrClampAfterFunction(const FrClampAfterFunction &other);

    FrClampAfterFunction *Clone() const override;

    std::string GetRepr() const override;

   protected:
    void Eval(double x) const override;

  };

  class FrClampBeforeFunction : public FrClampFunction {

   public:
    FrClampBeforeFunction(const FrFunctionBase &function, double xClamp);

    FrClampBeforeFunction(const FrClampBeforeFunction &other);

    FrClampBeforeFunction *Clone() const override;

    std::string GetRepr() const override;

   protected:
    void Eval(double x) const override;

  };


  FrClampAfterFunction clamp_after(const FrFunctionBase &function, double xClamp);

  FrClampBeforeFunction clamp_before(const FrFunctionBase &function, double xClamp);


}  // end namespace frydom



#endif //FRYDOM_FRCLAMPFUNCTION_H
