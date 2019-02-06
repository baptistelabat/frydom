//
// Created by frongere on 06/02/19.
//

#include "FrRamp.h"


namespace frydom {

    FrRamp_::FrRamp_() {
        m_chronoFunction = std::make_shared<chrono::ChFunction_Ramp>();
    }

    void FrRamp_::SetY0(double intercept) {
        GetChronoElement()->Set_y0(intercept);
    }

    void FrRamp_::SetSlope(double slope) {
        GetChronoElement()->Set_ang(slope);
    }

    void FrRamp_::Set(double intercept, double slope) {
        auto chronoElement = GetChronoElement();
        chronoElement->Set_y0(intercept);
        chronoElement->Set_ang(slope);
    }

    void FrRamp_::SetIsLimited(bool limit) {
        m_isLimited = limit;
        if (limit) Initialize();
    }

    bool FrRamp_::GetIsLimited() const {
        return m_isLimited;
    }

    void FrRamp_::SetXLimits(double xmin, double xmax) {
        m_xmin = xmin;
        m_xmax = xmax;
        m_isLimited = true;
        Initialize();
    }

    void FrRamp_::Initialize() {
        if (m_isLimited) {
            c_ymin = m_chronoFunction->Get_y(m_xmin);
            c_ymax = m_chronoFunction->Get_y(m_xmax);
        }
    }

    chrono::ChFunction_Ramp *FrRamp_::GetChronoElement() {
        return dynamic_cast<chrono::ChFunction_Ramp*>(m_chronoFunction.get());
    }

    double FrRamp_::Get_y(double x) const {
        if (m_isLimited) {
            if (x <= m_xmin) {
                return c_ymin;
            } else if (x >= m_xmax) {
                return c_ymax;
            } else {
                return FrFunction_::Get_y(x);
            }
        } else {
            return FrFunction_::Get_y(x);
        }
    }

    double FrRamp_::Get_y_dx(double x) const {
        if (m_isLimited) {
            if (x <= m_xmin) {
                return 0.;
            } else if (x >= m_xmax) {
                return 0.;
            } else {
                return FrFunction_::Get_y_dx(x);
            }
        } else {
            return FrFunction_::Get_y_dx(x);
        }
    }

    double FrRamp_::Get_y_dxdx(double x) const {
        if (m_isLimited) {
            if (x <= m_xmin) {
                return 0.;
            } else if (x >= m_xmax) {
                return 0.;
            } else {
                return FrFunction_::Get_y_dxdx(x);
            }
        } else {
            return FrFunction_::Get_y_dxdx(x);
        }
    }

}  // end namespace frydom
