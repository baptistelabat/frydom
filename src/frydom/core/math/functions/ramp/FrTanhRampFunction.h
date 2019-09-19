//
// Created by frongere on 07/02/19.
//

#ifndef FRYDOM_FRTANHRAMP_H
#define FRYDOM_FRTANHRAMP_H

#include "frydom/core/math/functions/FrFunctionBase.h"

namespace frydom {

    class FrTanhRampFunction : public FrFunctionBase {

    private:

        double m_tolerance = 1e-3;

        double m_width = 1.;
        double m_height = 1.;
        double m_yOffset = 0.; // TODO : placer dans la classe de base !!

        double c_a;

    public:

        FrTanhRampFunction();

        FrTanhRampFunction(const FrTanhRampFunction& other);

        FrTanhRampFunction* Clone() const;

        void SetTolerance(double tolerance);

        void SetWidth(double width);

        void SetHeight(double height);

        void SetYOffset(double yOffset);

        void SetByTwoPoints(double x0, double y0, double x1, double y1);

        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "TanhRampFunction"; }

    private:
        void ComputeA();

    protected:

        void Eval(double x) const override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRTANHRAMP_H
