//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRINTERP1D_H
#define FRYDOM_FRINTERP1D_H


#include <vector>
#include <memory>
#include <cassert>
#include <algorithm>
#include <complex>

namespace frydom {

    enum Interp1dMethod {
        LINEAR
    };


    template <class Scalar=double>
    class FrInterp1d {

    protected:
        std::shared_ptr<const std::vector<Scalar>> xcoord;
        std::shared_ptr<const std::vector<Scalar>> yval;
        unsigned long ndata = 0;
        Scalar xmin;
        Scalar xmax;

    public:
        FrInterp1d() {};
        ~FrInterp1d() {};

        // TODO: voir a separer l'implementation et la mettre en fin de fichier (pas directement dans le corps de la classe)

        virtual void Initialize(std::shared_ptr<const std::vector<Scalar>> x,
                                std::shared_ptr<const std::vector<Scalar>> y);

        virtual Scalar Eval(const Scalar x) const = 0;

        virtual std::vector<Scalar> Eval(const std::vector<Scalar>& xvector) const = 0;

        Scalar operator() (const Scalar x) const {
            return Eval(x);
        }

        std::vector<Scalar> operator() (const std::vector<Scalar>& xvector) const { return Eval(xvector); }

        static FrInterp1d<Scalar>* MakeInterp1d(Interp1dMethod method);

    };

    template <class Scalar>
    void FrInterp1d<Scalar>::Initialize(std::shared_ptr<const std::vector<Scalar>> x,
                                      std::shared_ptr<const std::vector<Scalar>> y) {

        assert( x->size() == y->size() );
        assert (std::is_sorted(x->begin(), x->end()));

        ndata = x->size();
        xmin = x->at(0);
        xmax = x->at(ndata-1);

        xcoord = x;
        yval = y;

    }


    template <class Scalar=double>
    class FrInterp1dLinear : public FrInterp1d<Scalar> {

    private:
        std::vector<Scalar> a;
        std::vector<Scalar> b;

    public:

        void Initialize(const std::shared_ptr<const std::vector<Scalar>> x,
                        const std::shared_ptr<const std::vector<Scalar>> y) override;

        Scalar Eval(const Scalar x) const;

        std::vector<Scalar> Eval(const std::vector<Scalar>& xvector) const;

    };

    template <class Scalar>
    void FrInterp1dLinear<Scalar>::Initialize(const std::shared_ptr<const std::vector<Scalar>> x,
                                            const std::shared_ptr<const std::vector<Scalar>> y) {

        FrInterp1d<Scalar>::Initialize(x, y);

        a.reserve(this->ndata);
        b.reserve(this->ndata);

        Scalar xi, xii, yi, yii, xii_m_xi;
        for (unsigned int i=1; i < this->ndata; ++i) {

            xi = this->xcoord->at(i-1);
            xii = this->xcoord->at(i);

            yi = this->yval->at(i-1);
            yii = this->yval->at(i);

            xii_m_xi = xii - xi;

            a.push_back( (yii-yi) / xii_m_xi );
            b.push_back( (yi*xii - xi*yii) / xii_m_xi );

        }
    }

    template <class Scalar>
    Scalar FrInterp1dLinear<Scalar>::Eval(const Scalar x) const {
        assert (x >= this->xmin &&
                x <= this->xmax);

        // First, binary search on the x coords
        auto upper = std::lower_bound(this->xcoord->begin(), this->xcoord->end(), x);
        auto index = std::distance(this->xcoord->begin(), upper);

        if (index == 0) index = 1;  // Bug fix for x == xmin

        Scalar a_ = a.at(index-1);
        Scalar b_ = b.at(index-1);

        return a_*x + b_;
    }

    template <class Scalar>
    std::vector<Scalar> FrInterp1dLinear<Scalar>::Eval(const std::vector<Scalar> &xvector) const {

        auto n = xvector.size();

        std::vector<Scalar> out;
        out.reserve(n);

        for (int i=0; i<n; i++) {
            out.push_back(Eval(xvector[i]));  // No boundchecking done here for performance as we are safe
        }
        return out;
    }

    /// Factory method to create 1D interpolation classes
    template <class Scalar>
    FrInterp1d<Scalar>* FrInterp1d<Scalar>::MakeInterp1d(Interp1dMethod method) {
        switch (method) {
            case LINEAR:
                return new FrInterp1dLinear<Scalar>;
            default:
                throw ("1D INTERPOLATION METHOD DOES NOT EXIST");
        }
    }

}  // end namespace frydom


#endif //FRYDOM_FRINTERP1D_H
