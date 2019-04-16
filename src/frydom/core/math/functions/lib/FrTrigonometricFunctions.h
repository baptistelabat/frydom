//
// Created by frongere on 12/02/19.
//

#ifndef FRYDOM_FRTRIGONOMETRICFUNCTIONS_H
#define FRYDOM_FRTRIGONOMETRICFUNCTIONS_H

#include "frydom/core/math/functions/FrFunctionBase.h"

namespace frydom {


    class FrCosFunction : public FrFunctionBase {

    public:
        explicit FrCosFunction(double alpha);
        FrCosFunction(const FrFunctionBase& function);
        FrCosFunction(const FrCosFunction& other);
        FrCosFunction* Clone() const override;

        std::string GetRepr() const override;

        std::string GetTypeName() const override { return "CosFunction"; }

    protected:
        void Eval(double x) const override;

    };

    FrCosFunction cos(const FrFunctionBase& function);




    class FrSinFunction : public FrFunctionBase {

    public:
        explicit FrSinFunction(double alpha);
        FrSinFunction(const FrFunctionBase& function);
        FrSinFunction(const FrSinFunction& other);
        FrSinFunction* Clone() const override;

        std::string GetRepr() const override;

        std::string GetTypeName() const override { return "SinFunction"; }

    protected:
        void Eval(double x) const override;

    };

    FrSinFunction sin(const FrFunctionBase& function);




    class FrTanFunction : public FrFunctionBase {

    public:
        explicit FrTanFunction(double alpha);
        FrTanFunction(const FrFunctionBase& function);
        FrTanFunction(const FrTanFunction& other);
        FrTanFunction* Clone() const override;

        std::string GetRepr() const override;

        std::string GetTypeName() const override { return "TanFunction"; }

    protected:
        void Eval(double x) const override;

    };

    FrTanFunction tan(const FrFunctionBase& function);


    /*
     * Inverse trigonometric functions
     */

    class FrACosFunction : public FrFunctionBase {

    public:
        explicit FrACosFunction(double alpha);
        FrACosFunction(const FrFunctionBase& function);
        FrACosFunction(const FrACosFunction& other);
        FrACosFunction* Clone() const override;

        std::string GetRepr() const override;

        std::string GetTypeName() const override { return "ACosFunction"; }

    protected:
        void Eval(double x) const override;

    };

    FrACosFunction acos(const FrFunctionBase& function);




    class FrASinFunction : public FrFunctionBase {

    public:
        explicit FrASinFunction(double alpha);
        FrASinFunction(const FrFunctionBase& function);
        FrASinFunction(const FrASinFunction& other);
        FrASinFunction* Clone() const override;

        std::string GetRepr() const override;

        std::string GetTypeName() const override { return "ASinFunction"; }

    protected:
        void Eval(double x) const override;

    };

    FrASinFunction asin(const FrFunctionBase& function);




    class FrATanFunction : public FrFunctionBase {

    public:
        explicit FrATanFunction(double alpha);
        FrATanFunction(const FrFunctionBase& function);
        FrATanFunction(const FrATanFunction& other);
        FrATanFunction* Clone() const override;

        std::string GetRepr() const override;

        std::string GetTypeName() const override { return "ATanFunction"; }

    protected:
        void Eval(double x) const override;

    };

    FrATanFunction atan(const FrFunctionBase& function);








}  // end namespace frydom



#endif //FRYDOM_FRTRIGONOMETRICFUNCTIONS_H
