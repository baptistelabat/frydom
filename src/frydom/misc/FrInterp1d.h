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

// TODO: differencier le x du y... ie x est real et y est real ou complex

namespace frydom {

    enum Interp1dMethod {
        LINEAR
    };


    template <class XReal, class YReal>
    class FrInterp1d {

    protected:
        std::shared_ptr<const std::vector<XReal>> xcoord;
        std::shared_ptr<const std::vector<YReal>> yval;
        unsigned long ndata = 0;
        XReal xmin;
        XReal xmax;

    public:
        FrInterp1d() {};
        ~FrInterp1d() {};

        // TODO: voir a separer l'implementation et la mettre en fin de fichier (pas directement dans le corps de la classe)

        virtual void Initialize(std::shared_ptr<const std::vector<XReal>> x,
                                std::shared_ptr<const std::vector<YReal>> y);

        virtual YReal Eval(const XReal x) const = 0;

        virtual std::vector<YReal> Eval(const std::vector<XReal>& xvector) const = 0;

        YReal operator() (const XReal x) const {
            return Eval(x);
        }

        std::vector<YReal> operator() (const std::vector<XReal>& xvector) const { return Eval(xvector); }

        static FrInterp1d<XReal, YReal>* MakeInterp1d(Interp1dMethod method);

    };

    template <class XReal, class YReal>
    void FrInterp1d<XReal, YReal>::Initialize(std::shared_ptr<const std::vector<XReal>> x,
                                              std::shared_ptr<const std::vector<YReal>> y) {

        assert( x->size() == y->size() );
        assert (std::is_sorted(x->begin(), x->end()));

        ndata = x->size();
        xmin = x->at(0);
        xmax = x->at(ndata-1);

        xcoord = x;
        yval = y;

    }


    template <class XReal, class YReal>
    class FrInterp1dLinear : public FrInterp1d<XReal, YReal> {

    private:
        std::vector<YReal> a;
        std::vector<YReal> b;

    public:

        void Initialize(const std::shared_ptr<const std::vector<XReal>> x,
                        const std::shared_ptr<const std::vector<YReal>> y) override;

        YReal Eval(const XReal x) const;

        std::vector<YReal> Eval(const std::vector<XReal>& xvector) const;

    };

    template <class XReal, class YReal>
    void FrInterp1dLinear<XReal, YReal>::Initialize(const std::shared_ptr<const std::vector<XReal>> x,
                                                    const std::shared_ptr<const std::vector<YReal>> y) {

        FrInterp1d<XReal, YReal>::Initialize(x, y);

        a.reserve(this->ndata);
        b.reserve(this->ndata);

        XReal xi, xii, xii_m_xi;
        YReal yi, yii;
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

    template <class XReal, class YReal>
    YReal FrInterp1dLinear<XReal, YReal>::Eval(const XReal x) const {
        // TODO: il faut que le type de retour soit compatible avec real et complex !!!
        assert (x >= this->xmin &&
                x <= this->xmax);

        // First, binary search on the x coords
        auto upper = std::lower_bound(this->xcoord->begin(), this->xcoord->end(), x);
        auto index = std::distance(this->xcoord->begin(), upper);

        if (index == 0) index = 1;  // Bug fix for x == xmin

        YReal a_ = a.at(index-1);
        YReal b_ = b.at(index-1);

        return a_*x + b_;
    }

    template <class XReal, class YReal>
    std::vector<YReal> FrInterp1dLinear<XReal, YReal>::Eval(const std::vector<XReal> &xvector) const {

        auto n = xvector.size();

        std::vector<YReal> out;
        out.reserve(n);

        for (int i=0; i<n; i++) {
            out.push_back(Eval(xvector[i]));  // No boundchecking done here for performance as we are safe
        }
        return out;
    }

    /// Factory method to create 1D interpolation classes
    template <class XReal, class YReal>
    FrInterp1d<XReal, YReal>* FrInterp1d<XReal, YReal>::MakeInterp1d(Interp1dMethod method) {
        switch (method) {
            case LINEAR:
                return new FrInterp1dLinear<XReal, YReal>;
            default:
                throw ("1D INTERPOLATION METHOD DOES NOT EXIST");
        }
    }

}  // end namespace frydom


#endif //FRYDOM_FRINTERP1D_H
