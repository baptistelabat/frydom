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

#include <MathUtils/Constants.h>
#include "FrCosRampFunction.h"
#include "frydom/core/math/functions/lib/FrClampFunction.h"
#include "frydom/core/math/functions/lib/FrTrigonometricFunctions.h"

namespace frydom{


    frydom::FrCosRampFunction::FrCosRampFunction() {
        SetByTwoPoints(m_x0, m_y0, m_x1, m_y1);
    }

    FrCosRampFunction::FrCosRampFunction(double x0, double y0, double x1, double y1) : FrFunctionBase() {
        SetByTwoPoints(x0, y0, x1, y1);
    }

    FrCosRampFunction::FrCosRampFunction(const frydom::FrCosRampFunction &other) : FrFunctionBase(other) {}

    FrCosRampFunction* FrCosRampFunction::Clone() const {
        return new FrCosRampFunction(*this);
    }

    void FrCosRampFunction::SetByTwoPoints(double x0, double y0, double x1, double y1) {

        assert(x0<x1);

        m_x0 = x0;
        m_x1 = x1;
        m_y0 = y0;
        m_y1 = y1;

        double amplitude = 0.5*(y1 - y0);
        double circularFreq = MU_PI / (x1- x0);

        m_function = clamp_after(clamp_before(y0 + amplitude * (1.-cos(circularFreq*(FrVarXFunction()-x0))), x0), x1).Clone();
    }

    void FrCosRampFunction::GetByTwoPoints(double &x0, double &y0, double &x1, double &y1) {

        x0 = m_x0;
        x1 = m_x1;
        y0 = m_y0;
        y1 = m_y1;

    }

    void FrCosRampFunction::Initialize() {

    }

    std::string FrCosRampFunction::GetRepr() const {
        fmt::MemoryWriter mw;

//        mw.write("Ramp(({:.3g}, {:.3g}), ({:.3g}, {:.3g}))", m_x0, Get_y(m_x0), m_x1, Get_y(m_x1));
//        return mw.str();
        return ""; // TODO
    }

    void FrCosRampFunction::Eval(double x) const {

        if (IsEval(x)) return;

        c_x = x;
        c_y = m_function->Get_y(x);
        c_y_dx = m_function->Get_y_dx(x);
        c_y_dxdx = m_function->Get_y_dxdx(x);

    }

} //end namespace frydom

