// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#ifndef FRYDOM_FRRAMP_H
#define FRYDOM_FRRAMP_H

#include "frydom/core/math/functions/FrFunctionBase.h"


namespace frydom {


    /// Class for representing a ramp
    ///
    ///             --------
    ///            /
    ///           /
    ///          /
    /// --------
    ///
    ///
    class FrLinearRampFunction : public FrFunctionBase {

    private:

//        double m_slope = 1.;
//        double m_intercept = 0.;
//
//        double m_x0 = 0.; // TODO : plutot reposer sur l'offset de la fonction de base !
//        double m_x1 = 1.;

        double m_x0 = 0., m_x1 = 1., m_y0 = 0., m_y1 = 1.;

    public:

        FrLinearRampFunction();

        FrLinearRampFunction(double x0, double y0, double x1, double y1);

        FrLinearRampFunction(const FrLinearRampFunction& other);

        FrLinearRampFunction* Clone() const override;

//        void SetY0(double intercept);
//
//        double GetY0() const;
//
//        void SetSlope(double slope);
//
//        double GetSlope() const;
//
//        void SetInterceptAndSlope(double intercept, double slope);
//
//        void SetXWindow(double x0, double x1);
//
        void SetByTwoPoints(double x0, double y0, double x1, double y1);
        
        void GetByTwoPoints(double& x0, double& y0, double& x1, double& y1 );

        void Initialize() override;

        std::string GetRepr() const override;

        std::string GetTypeName() const override { return "LinearRampFunction"; }

    protected:
        void Eval(double x) const override;



    };


}  // end namespace frydom

#endif //FRYDOM_FRRAMP_H
