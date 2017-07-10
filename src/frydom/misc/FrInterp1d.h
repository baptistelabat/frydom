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

    enum Interp1dMethod {
        LINEAR
    };


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

        static FrInterp1d<Real>* MakeInterp1d(Interp1dMethod method);

    };


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

    /// Factory method to create 1D interpolation classes
    template <class Real>
    FrInterp1d<Real>* FrInterp1d<Real>::MakeInterp1d(Interp1dMethod method) {
        switch (method) {
            case LINEAR:
                return new FrInterp1dLinear<Real>;
            default:
                throw ("1D INTERPOLATION METHOD DOES NOT EXIST");
        }
    }



}  // end namespace frydom


#endif //FRYDOM_FRINTERP1D_H
