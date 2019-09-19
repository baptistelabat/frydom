//
// Created by frongere on 11/02/19.
//

#ifndef FRYDOM_FRPOLYNOMIALFUNCITON_H
#define FRYDOM_FRPOLYNOMIALFUNCITON_H

#include <map>

#include "frydom/core/math/functions/FrFunctionBase.h"

namespace frydom {

    class FrPolynomialFunction : public FrFunctionBase {

//    private:
//        std::map<unsigned int, double> m_poly;
//        unsigned int m_order = 0;

    public:
        FrPolynomialFunction(double constant);
        FrPolynomialFunction(const FrPolynomialFunction& other);
        FrPolynomialFunction* Clone() const override;

        void Add(double val, unsigned int power);

        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "PolynomialFunction"; }


    protected:
        void Eval(double x) const override;




    };

}  // end namespace frydom


#endif //FRYDOM_FRPOLYNOMIALFUNCITON_H
