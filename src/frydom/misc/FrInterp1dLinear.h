//
// Created by frongere on 04/07/17.
//

#ifndef FRYDOM_FRINTERP1DLINEAR_H
#define FRYDOM_FRINTERP1DLINEAR_H

#include <iostream>
#include "FrInterp1d.h"

namespace frydom {

    template <class Real=double>
    class FrInterp1dLinear : public FrInterp1d<Real> {

    private:
        std::vector<Real> a;
        std::vector<Real> b;

    public:

        // TODO: voir a separer l'implementation et la mettre en fin de fichier (pas directement dans le corps de la classe)
        void Initialize(const std::shared_ptr<std::vector<Real>> x,
                        const std::shared_ptr<std::vector<Real>> y) {

            FrInterp1d<Real>::Initialize(x, y);

            Real xi, xii, yi, yii, xii_m_xi;
            for (unsigned int i=1; i < this->ndata; i++) {

                xi = this->xcoord->at(i-1);
                xii = this->xcoord->at(i);

                yi = this->yval->at(i-1);
                yii = this->yval->at(i);

                xii_m_xi = xii - xi;

                a.push_back( (yii-yi) / xii_m_xi );
                b.push_back( (yi*xii - xi*yii) / xii_m_xi );

            }
        }

        Real Eval(const Real x) const {
            assert (x >= this->xmin &&
                    x <= this->xmax);

            // First, binary search on the x coords
            auto upper = std::upper_bound(this->xcoord->begin(), this->xcoord->end(), x);
            auto index = std::distance(this->xcoord->begin(), upper);

            // FIXME: bug quand on s'approche de la borne sup...
            Real a_ = a.at(index-1);
            Real b_ = b.at(index-1);

            return a_*x + b_;

        }

        std::vector<Real> Eval(const std::vector<Real> xvector) const {
            std::vector<Real> out;
            Real val;

            auto n = xvector.size();
            for (int i=0; i<n; i++) {
                val = Eval(xvector.at(i));
                out.push_back(val);
            }
            return out;
        };

    };


}  // end namespace frydom

#endif //FRYDOM_FRINTERP1DLINEAR_H
