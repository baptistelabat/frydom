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

        // TODO: voir a separer l'implementation et la mettre en fin de fichier (pas directement dans le corps de la classe)

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

        virtual Real Eval(const Real x) const = 0;

        virtual std::vector<Real> Eval(const std::vector<Real> xvector) const = 0;

        Real operator() (const Real x) const {
            return Eval(x);
        }

        std::vector<Real> operator() (const std::vector<Real> xvector) const {
            return Eval(xvector);
        }

    };

}  // end namespace frydom


#endif //FRYDOM_FRINTERP1D_H
