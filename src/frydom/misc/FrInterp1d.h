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


    template <class XReal, class YScalar>
    class FrInterp1d {

    protected:
        std::shared_ptr<const std::vector<XReal>> xcoord;
        std::shared_ptr<const std::vector<YScalar>> yval;
        unsigned long ndata = 0;
        XReal xmin;
        XReal xmax;

    public:
        // TODO: voir a separer l'implementation et la mettre en fin de fichier (pas directement dans le corps de la classe)

        virtual void Initialize(std::shared_ptr<const std::vector<XReal>> x,
                                std::shared_ptr<const std::vector<YScalar>> y);

        virtual YScalar Eval(XReal x) const = 0;

        virtual std::vector<YScalar> Eval(const std::vector<XReal>& xvector) const = 0;

        YScalar operator() (const XReal x) const {
            return Eval(x);
        }

        std::vector<YScalar> operator() (const std::vector<XReal>& xvector) const { return Eval(xvector); }

        static FrInterp1d<XReal, YScalar>* MakeInterp1d(Interp1dMethod method);

    };

    template <class XReal, class YScalar>
    void FrInterp1d<XReal, YScalar>::Initialize(std::shared_ptr<const std::vector<XReal>> x,
                                              std::shared_ptr<const std::vector<YScalar>> y) {

        assert( x->size() == y->size() );
        assert (std::is_sorted(x->begin(), x->end()));

        ndata = x->size();
        xmin = x->at(0);
        xmax = x->at(ndata-1);

        xcoord = x;
        yval = y;

    }


    template <class XReal, class YScalar>
    class FrInterp1dLinear : public FrInterp1d<XReal, YScalar> {

    private:
        std::vector<YScalar> a;
        std::vector<YScalar> b;

    public:

        void Initialize(std::shared_ptr<const std::vector<XReal>> x,
                        std::shared_ptr<const std::vector<YScalar>> y) override;

        YScalar Eval(XReal x) const;

        std::vector<YScalar> Eval(const std::vector<XReal>& xvector) const;

    };

    template <class XReal, class YScalar>
    void FrInterp1dLinear<XReal, YScalar>::Initialize(const std::shared_ptr<const std::vector<XReal>> x,
                                                    const std::shared_ptr<const std::vector<YScalar>> y) {

        FrInterp1d<XReal, YScalar>::Initialize(x, y);

        a.reserve(this->ndata);
        b.reserve(this->ndata);

        XReal xi, xii, xii_m_xi;
        YScalar yi, yii;
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

    template <class XReal, class YScalar>
    YScalar FrInterp1dLinear<XReal, YScalar>::Eval(const XReal x) const {
        // TODO: il faut que le type de retour soit compatible avec real et complex !!!
        assert (x >= this->xmin &&
                x <= this->xmax);

        // First, binary search on the x coords
        auto upper = std::lower_bound(this->xcoord->begin(), this->xcoord->end(), x);
        auto index = std::distance(this->xcoord->begin(), upper);

        if (index == 0) index = 1;  // Bug fix for x == xmin

        YScalar a_ = a.at(index-1);
        YScalar b_ = b.at(index-1);

        return a_*x + b_;
    }

    template <class XReal, class YScalar>
    std::vector<YScalar> FrInterp1dLinear<XReal, YScalar>::Eval(const std::vector<XReal> &xvector) const {

        auto n = xvector.size();

        std::vector<YScalar> out;
        out.reserve(n);

        for (int i=0; i<n; i++) {
            out.push_back(Eval(xvector[i]));  // No boundchecking done here for performance as we are safe
        }
        return out;
    }

    /// Factory method to create 1D interpolation classes
    template <class XReal, class YScalar>
    FrInterp1d<XReal, YScalar>* FrInterp1d<XReal, YScalar>::MakeInterp1d(Interp1dMethod method) {
        switch (method) {
            case LINEAR:
                return new FrInterp1dLinear<XReal, YScalar>;
            default:
                throw ("1D INTERPOLATION METHOD DOES NOT EXIST");
        }
    }

}  // end namespace frydom


#endif //FRYDOM_FRINTERP1D_H
