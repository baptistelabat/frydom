//
// Created by frongere on 12/02/19.
//

#ifndef FRYDOM_FRHYPERBOLICTRIGONOMETRICFUNCTIONS_H
#define FRYDOM_FRHYPERBOLICTRIGONOMETRICFUNCTIONS_H


#include "frydom/core/math/functions/FrFunctionBase.h"


namespace frydom {


    class FrCosHFunction : public FrFunctionBase {

    public:
        explicit FrCosHFunction(double alpha);
        FrCosHFunction(const FrFunctionBase& function);
        FrCosHFunction(const FrCosHFunction& other);
        FrCosHFunction* Clone() const override;

        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

    FrCosHFunction cosh(const FrFunctionBase& function);




    class FrSinHFunction : public FrFunctionBase {

    public:
        explicit FrSinHFunction(double alpha);
        FrSinHFunction(const FrFunctionBase& function);
        FrSinHFunction(const FrSinHFunction& other);
        FrSinHFunction* Clone() const override;

        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

    FrSinHFunction sinh(const FrFunctionBase& function);




    class FrTanHFunction : public FrFunctionBase {

    public:
        explicit FrTanHFunction(double alpha);
        FrTanHFunction(const FrFunctionBase& function);
        FrTanHFunction(const FrTanHFunction& other);
        FrTanHFunction* Clone() const override;

        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

    FrTanHFunction tanh(const FrFunctionBase& function);


    /*
     * Inverse hyperbolic trigonometric functions
     */

    class FrACosHFunction : public FrFunctionBase {

    public:
        explicit FrACosHFunction(double alpha);
        FrACosHFunction(const FrFunctionBase& function);
        FrACosHFunction(const FrACosHFunction& other);
        FrACosHFunction* Clone() const override;

        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

    FrACosHFunction acosh(const FrFunctionBase& function);




    class FrASinHFunction : public FrFunctionBase {

    public:
        explicit FrASinHFunction(double alpha);
        FrASinHFunction(const FrFunctionBase& function);
        FrASinHFunction(const FrASinHFunction& other);
        FrASinHFunction* Clone() const override;

        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

    FrASinHFunction asinh(const FrFunctionBase& function);




    class FrATanHFunction : public FrFunctionBase {

    public:
        explicit FrATanHFunction(double alpha);
        FrATanHFunction(const FrFunctionBase& function);
        FrATanHFunction(const FrATanHFunction& other);
        FrATanHFunction* Clone() const override;

        std::string GetRepr() const override;

    protected:
        void Eval(double x) const override;

    };

    FrATanHFunction atanh(const FrFunctionBase& function);






}  // end namespace frydom



#endif //FRYDOM_FRHYPERBOLICTRIGONOMETRICFUNCTIONS_H
