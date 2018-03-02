//
// Created by frongere on 04/07/17.
//

#ifndef FRYDOM_FRINTERP1DLINEAR_H
#define FRYDOM_FRINTERP1DLINEAR_H

#include <iostream>
#include "FrInterp1d.h"

namespace frydom {

    template <class Real=double>
    class FrInterp1DLinear : public FrInterp1d<Real> {

    private:
        std::vector<Real> a;
        std::vector<Real> b;

    public:

        void Initialize(const std::shared_ptr<std::vector<Real>> x,
                        const std::shared_ptr<std::vector<Real>> y) {

            FrInterp1d<Real>::Initialize(x, y);

//            unsigned long n = x->size();
            for (unsigned int i=1; i < this->ndata; i++) {

                Real xi = this->xcoord->at(i-1);
                Real xii = this->xcoord->at(i);

                Real yi = this->yval->at(i-1);
                Real yii = this->yval->at(i);

                Real xii_m_xi = xii - xi;

                a.push_back( (yii-yi) / xii_m_xi );
                b.push_back( (yi*xii - xi*yii) / xii_m_xi );

            }
        }

        Real Eval(Real x) {
            assert (x >= this->xmin &&
                    x <= this->xmax);

            // First, binary search on the
            auto lower = std::lower_bound(this->xcoord->begin(), this->xcoord->end(), x);
            auto index = std::distance(this->xcoord->begin(), lower);

            Real a_ = a.at(index-1);
            Real b_ = b.at(index-1);

            return a_*x + b_;

        }

        std::vector<Real> Eval(const std::vector<Real> xvector) {};

    };


}  // end namespace frydom

#endif //FRYDOM_FRINTERP1DLINEAR_H
