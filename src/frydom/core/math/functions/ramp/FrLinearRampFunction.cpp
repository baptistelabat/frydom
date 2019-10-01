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

#include "FrLinearRampFunction.h"
#include "frydom/core/math/functions/lib/FrClampFunction.h"
#include "fmt/format.h"

namespace frydom {

    FrLinearRampFunction::FrLinearRampFunction() {
        SetByTwoPoints(m_x0, m_y0, m_x1, m_y1);
    }

    FrLinearRampFunction::FrLinearRampFunction(double x0, double y0, double x1, double y1) : FrFunctionBase() {
        SetByTwoPoints(x0, y0, x1, y1);
    }

    FrLinearRampFunction::FrLinearRampFunction(const frydom::FrLinearRampFunction &other) : FrFunctionBase(other) {}

    FrLinearRampFunction* FrLinearRampFunction::Clone() const {
        return new FrLinearRampFunction(*this);
    }
//
    void FrLinearRampFunction::SetByTwoPoints(double x0, double y0, double x1, double y1) {

        assert(x0<x1);

        m_x0 = x0;
        m_x1 = x1;
        m_y0 = y0;
        m_y1 = y1;

        double slope = (y1-y0) / (x1-x0);
        double intercept = (y0 - slope*x0);
        m_function = clamp_after(clamp_before((slope*FrVarXFunction() + intercept), x0), x1).Clone();
    }

    void FrLinearRampFunction::GetByTwoPoints(double &x0, double &y0, double &x1, double &y1) {

        x0 = m_x0;
        x1 = m_x1;
        y0 = m_y0;
        y1 = m_y1;

    }

    void FrLinearRampFunction::Initialize() {

    }

    std::string FrLinearRampFunction::GetRepr() const {
        fmt::MemoryWriter mw;

        return ""; // TODO
    }

    void FrLinearRampFunction::Eval(double x) const {

        if (IsEval(x)) return;

        c_x = x;
        c_y = m_function->Get_y(x);
        c_y_dx = m_function->Get_y_dx(x);
        c_y_dxdx = m_function->Get_y_dxdx(x);

    }


}  // end namespace frydom
