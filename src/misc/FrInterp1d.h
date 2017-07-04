//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRINTERP1D_H
#define FRYDOM_FRINTERP1D_H


#include <vector>
#include <memory>
#include <cassert>

namespace frydom {

    template <class Real=double>
    class FrInterp1d {

    private:
        std::unique_ptr<std::vector<Real>> xcoord;
        std::unique_ptr<std::vector<Real>> yval;

    public:
        FrInterp1d() {};
        ~FrInterp1d() {};

        virtual void Initialize(std::vector<Real>* x,
                                std::vector<Real>* y) {
            assert( x->size() == y->size() );
            xcoord.reset(x);
            yval.reset(y);
        }

        virtual Real Eval(Real x) = 0;

        virtual std::vector<Real> Eval(const std::vector<Real> xvector) = 0;

    };

}  // end namespace frydom


#endif //FRYDOM_FRINTERP1D_H
