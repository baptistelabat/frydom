//
// Created by frongere on 12/02/19.
//

#include "FrClampFunction.h"


namespace frydom {


  FrClampFunction::FrClampFunction(const FrFunctionBase &function, double xClamp) :
      FrFunctionBase(function), m_xClamp(xClamp) {
    m_function = function.Clone();
  }

  FrClampFunction::FrClampFunction(const FrClampFunction &other) : FrFunctionBase(other) {
    m_xClamp = other.m_xClamp;
  }

  void FrClampFunction::Set(double xClamp) {
    m_xClamp = xClamp;
  }


  FrClampAfterFunction::FrClampAfterFunction(const FrFunctionBase &function, double xClamp) : FrClampFunction(function,
                                                                                                              xClamp) {}

  FrClampAfterFunction::FrClampAfterFunction(const FrClampAfterFunction &other) : FrClampFunction(other) {}

  FrClampAfterFunction *FrClampAfterFunction::Clone() const {
    return new FrClampAfterFunction(*this);
  }

  std::string FrClampAfterFunction::GetRepr() const {
    return ""; // TODO
  }

  void FrClampAfterFunction::Eval(double x) const {

    if (x >= m_xClamp) {
      if (IsEval(m_xClamp)) return;
      c_x = m_xClamp;
      c_y = m_function->Get_y(c_x);
      c_y_dx = 0.;
      c_y_dxdx = 0.;
      return;
    }

    if (IsEval(x)) return;

    c_x = x;
    c_y = m_function->Get_y(x);
    c_y_dx = m_function->Get_y_dx(x);
    c_y_dxdx = m_function->Get_y_dxdx(x);

  }


  FrClampBeforeFunction::FrClampBeforeFunction(const FrFunctionBase &function, double xClamp) : FrClampFunction(
      function, xClamp) {}

  FrClampBeforeFunction::FrClampBeforeFunction(const FrClampBeforeFunction &other) : FrClampFunction(other) {}

  FrClampBeforeFunction *FrClampBeforeFunction::Clone() const {
    return new FrClampBeforeFunction(*this);
  }

  std::string FrClampBeforeFunction::GetRepr() const {
    return ""; // TODO
  }

  void FrClampBeforeFunction::Eval(double x) const {

    if (x <= m_xClamp) {
      if (IsEval(m_xClamp)) return;
      c_x = m_xClamp;
      c_y = m_function->Get_y(c_x);
      c_y_dx = 0.;
      c_y_dxdx = 0.;
      return;
    }

    if (IsEval(x)) return;

    c_x = x;
    c_y = m_function->Get_y(x);
    c_y_dx = m_function->Get_y_dx(x);
    c_y_dxdx = m_function->Get_y_dxdx(x);

  }


  FrClampAfterFunction clamp_after(const FrFunctionBase &function, double xClamp) {
    return FrClampAfterFunction(function, xClamp);
  }

  FrClampBeforeFunction clamp_before(const FrFunctionBase &function, double xClamp) {
    return FrClampBeforeFunction(function, xClamp);
  }


}  // end namespace frydom
