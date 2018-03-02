//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRINTERP1D_H
#define FRYDOM_FRINTERP1D_H


#include <vector>
#include <memory>
#include <cassert>
#include <algorithm>

namespace frydom {

    template <class Real=double>
    class FrInterp1d {

    protected:
        std::shared_ptr<std::vector<Real>> xcoord;
        std::shared_ptr<std::vector<Real>> yval;
        unsigned long ndata;
        Real xmin;
        Real xmax;

    public:
        FrInterp1d() {};
        ~FrInterp1d() {};

        virtual void Initialize(const std::shared_ptr<std::vector<Real>> x,
                                const std::shared_ptr<std::vector<Real>> y) {
            assert( x->size() == y->size() );
            assert (std::is_sorted(x->begin(), x->end()));
            ndata = x->size();
            xmin = x->at(0);
            xmax = x->at(ndata-1);

            xcoord = x;
            yval = y;

        }

        virtual Real Eval(Real x) = 0;

        virtual std::vector<Real> Eval(const std::vector<Real> xvector) = 0;

    };

}  // end namespace frydom


#endif //FRYDOM_FRINTERP1D_H
