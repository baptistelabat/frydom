//
// Created by frongere on 11/02/19.
//

#include "FrLinearFunction.h"
#include "FrPowFunction.h"


namespace frydom {

    FrLinearFunction::FrLinearFunction(double intercept, double slope) : m_intercept(intercept), m_slope(slope) {
//        *this = slope * FrPowFunction(1) + FrConstantFunction(intercept);
    }

    FrLinearFunction::FrLinearFunction(const FrLinearFunction& other) {
        m_intercept = other.m_intercept;
        m_slope = other.m_slope;

    }

    FrLinearFunction* FrLinearFunction::Clone() const {
        return new FrLinearFunction(*this);
    }

    void FrLinearFunction::SetByTwoPoints(double x0, double y0, double x1, double y1) {
        m_slope = (y1-y0) / (x1-x0);
        m_intercept = y0 - m_slope * x0;
    }

    void FrLinearFunction::SetSlope(double slope) {
        m_slope = slope;
    }

    double FrLinearFunction::GetSlope() const {
        return m_slope;
    }

    void FrLinearFunction::SetIntercept(double intercept) {
        m_intercept = intercept;
    }

    double FrLinearFunction::GetIntercept() const {
        return m_intercept;
    }

    void FrLinearFunction::Set(double intercept, double slope) {
        m_intercept = intercept;
        m_slope = slope;
    }

    std::string FrLinearFunction::GetRepr() const {
        return ""; //TODO
    }

    void FrLinearFunction::Eval(double x) const {

    }


}  // end namespace frydom
