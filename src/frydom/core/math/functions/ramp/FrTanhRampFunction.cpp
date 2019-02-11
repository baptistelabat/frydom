//
// Created by frongere on 07/02/19.
//

#include "FrTanhRampFunction.h"


namespace frydom {


    FrTanhRampFunction::FrTanhRampFunction() {
        ComputeA();
    }

    FrTanhRampFunction::FrTanhRampFunction(const FrTanhRampFunction& other) : FrFunction_() {
        m_tolerance = other.m_tolerance;
        m_width = other.m_width;
        m_height = other.m_height;
        m_yOffset = other.m_yOffset;
    }

    FrTanhRampFunction* FrTanhRampFunction::Clone() const {
        return new FrTanhRampFunction(*this);
    }

    void FrTanhRampFunction::SetTolerance(double tolerance) {
        m_tolerance = tolerance;
        ComputeA();
    }

    void FrTanhRampFunction::SetWidth(double width) {
        m_width = width;
    }

    void FrTanhRampFunction::SetHeight(double height) {
        m_height = height;
    }

    void FrTanhRampFunction::SetYOffset(double yOffset) {
        m_yOffset = yOffset;
    }

    void FrTanhRampFunction::SetByTwoPoints(double x0, double y0, double x1, double y1) {
        m_xOffset = x0;
        m_width = x1-x0;
        m_height = y1-y0;
        m_yOffset = y0;
    }

    void FrTanhRampFunction::ComputeA() {
        c_a = 0.5 * log((1-m_tolerance) / m_tolerance);
    }

    void FrTanhRampFunction::Eval(double x) const {
        double xTmp(x);
        if (xTmp <= m_xOffset) {
//            xTmp = m_xOffset;
            c_y = m_yOffset;
            c_y_dx = 0.;
            c_y_dxdx = 0.;
            return;
        } else if (xTmp >= m_xOffset + m_width) {
//            xTmp = m_xOffset + m_width;
            c_y = m_yOffset + m_height;
            c_y_dx = 0.;
            c_y_dxdx = 0.;
            return;
        }

        if (IsEval(xTmp)) return;

        c_x = xTmp;

        double alpha = 2.*c_a*c_x/m_width-c_a;
        double th = tanh(alpha);
        double ch = cosh(alpha);
        c_y = 0.5 * m_height * (1. + th) + m_yOffset;

        c_y_dx = (m_height*c_a/m_width) / (ch*ch);
//        c_y_dxdx = - 4.*m_height*c_a*c_a*th / (m_width*m_width);
        c_y_dxdx = 0.; // FIXME : la derivee seconde est mal calculee !

    }
}  // end namespace frydom
