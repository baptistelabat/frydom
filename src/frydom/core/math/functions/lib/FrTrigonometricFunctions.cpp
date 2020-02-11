//
// Created by frongere on 12/02/19.
//

#include "FrTrigonometricFunctions.h"


namespace frydom {

  /*
   * FrCosFunction
   */

  FrCosFunction::FrCosFunction(double alpha) {
    m_function = FrConstantFunction(alpha).Clone();
  }

  FrCosFunction::FrCosFunction(const FrFunctionBase &function) {
    m_function = function.Clone();
  }

  FrCosFunction::FrCosFunction(const FrCosFunction &other) : FrFunctionBase(other) {}

  FrCosFunction *FrCosFunction::Clone() const {
    return new FrCosFunction(*this);
  }

  std::string FrCosFunction::GetRepr() const {
    return ""; // TODO
  }

  void FrCosFunction::Eval(double x) const {
    if (IsEval(x)) return;

    c_x = x;
    double u = m_function->Get_y(x);
    double u_dx = m_function->Get_y_dx(x);
    double u_dxdx = m_function->Get_y_dxdx(x);
    double Su = std::sin(u);

    c_y = std::cos(u);
    c_y_dx = -u_dx * Su;
    c_y_dxdx = -u_dxdx * Su - u_dx * u_dx * c_y;

  }

  FrCosFunction cos(const FrFunctionBase &function) {
    return FrCosFunction(function);
  }


  /*
   * FrSinFunction
   */

  FrSinFunction::FrSinFunction(double alpha) {
    m_function = FrConstantFunction(alpha).Clone();
  }

  FrSinFunction::FrSinFunction(const FrFunctionBase &function) {
    m_function = function.Clone();
  }

  FrSinFunction::FrSinFunction(const FrSinFunction &other) : FrFunctionBase(other) {}

  FrSinFunction *FrSinFunction::Clone() const {
    return new FrSinFunction(*this);
  }

  std::string FrSinFunction::GetRepr() const {
    return ""; // TODO
  }

  void FrSinFunction::Eval(double x) const {
    if (IsEval(x)) return;

    c_x = x;
    double u = m_function->Get_y(x);
    double u_dx = m_function->Get_y_dx(x);
    double u_dxdx = m_function->Get_y_dxdx(x);
    double Cu = std::cos(u);

    c_y = std::sin(u);
    c_y_dx = u_dx * Cu;
    c_y_dxdx = u_dxdx * Cu - u_dx * u_dx * c_y;

  }

  FrSinFunction sin(const FrFunctionBase &function) {
    return FrSinFunction(function);
  }


  /*
   * FrTanFunction
   */

  FrTanFunction::FrTanFunction(double alpha) {
    m_function = FrConstantFunction(alpha).Clone();
  }

  FrTanFunction::FrTanFunction(const FrFunctionBase &function) {
    m_function = function.Clone();
  }

  FrTanFunction::FrTanFunction(const FrTanFunction &other) : FrFunctionBase(other) {}

  FrTanFunction *FrTanFunction::Clone() const {
    return new FrTanFunction(*this);
  }

  std::string FrTanFunction::GetRepr() const {
    return ""; // TODO
  }

  void FrTanFunction::Eval(double x) const {
    if (IsEval(x)) return;

    c_x = x;
    double u = m_function->Get_y(x);
    double u_dx = m_function->Get_y_dx(x);
    double u_dxdx = m_function->Get_y_dxdx(x);
    double tanu = std::tan(u);
    double tmp = 1 + tanu * tanu;

    c_y = tanu;
    c_y_dx = u_dx * tmp;
    c_y_dxdx = u_dxdx * tmp + 2. * u_dx * tanu * c_y_dx;

  }

  FrTanFunction tan(const FrFunctionBase &function) {
    return FrTanFunction(function);
  }


  /*
   * Invers trigonometric functions
   */

  FrACosFunction::FrACosFunction(double alpha) {
    m_function = FrConstantFunction(alpha).Clone();
  }

  FrACosFunction::FrACosFunction(const FrFunctionBase &function) {
    m_function = function.Clone();
  }

  FrACosFunction::FrACosFunction(const FrACosFunction &other) : FrFunctionBase(other) {}

  FrACosFunction *FrACosFunction::Clone() const {
    return new FrACosFunction(*this);
  }

  std::string FrACosFunction::GetRepr() const {
    return ""; // TODO
  }

  void FrACosFunction::Eval(double x) const {
    if (IsEval(x)) return;

    c_x = x;
    double u = m_function->Get_y(x);
    double u_dx = m_function->Get_y_dx(x);
    double u_dxdx = m_function->Get_y_dxdx(x);
    double tmp = 1. - u * u;
    double stmp = sqrt(tmp);

    c_y = std::acos(u);
    c_y_dx = -u_dx / stmp;
    c_y_dxdx = -u_dxdx / stmp - u * u_dx * u_dx / std::pow(tmp, 1.5);

  }

  FrACosFunction acos(const FrFunctionBase &function) {
    return FrACosFunction(function);
  }


  /*
   * FrSinFunction
   */

  FrASinFunction::FrASinFunction(double alpha) {
    m_function = FrConstantFunction(alpha).Clone();
  }

  FrASinFunction::FrASinFunction(const FrFunctionBase &function) {
    m_function = function.Clone();
  }

  FrASinFunction::FrASinFunction(const FrASinFunction &other) : FrFunctionBase(other) {}

  FrASinFunction *FrASinFunction::Clone() const {
    return new FrASinFunction(*this);
  }

  std::string FrASinFunction::GetRepr() const {
    return ""; // TODO
  }

  void FrASinFunction::Eval(double x) const {
    if (IsEval(x)) return;

    c_x = x;
    double u = m_function->Get_y(x);
    double u_dx = m_function->Get_y_dx(x);
    double u_dxdx = m_function->Get_y_dxdx(x);
    double tmp = 1. - u * u;
    double stmp = sqrt(tmp);

    c_y = std::asin(u);
    c_y_dx = u_dx / stmp;
    c_y_dxdx = u_dxdx / stmp + u * u_dx * u_dx / std::pow(tmp, 1.5);

  }

  FrASinFunction asin(const FrFunctionBase &function) {
    return FrASinFunction(function);
  }


  /*
   * FrTanFunction
   */

  FrATanFunction::FrATanFunction(double alpha) {
    m_function = FrConstantFunction(alpha).Clone();
  }

  FrATanFunction::FrATanFunction(const FrFunctionBase &function) {
    m_function = function.Clone();
  }

  FrATanFunction::FrATanFunction(const FrATanFunction &other) : FrFunctionBase(other) {}

  FrATanFunction *FrATanFunction::Clone() const {
    return new FrATanFunction(*this);
  }

  std::string FrATanFunction::GetRepr() const {
    return ""; // TODO
  }

  void FrATanFunction::Eval(double x) const {
    if (IsEval(x)) return;

    c_x = x;
    double u = m_function->Get_y(x);
    double u_dx = m_function->Get_y_dx(x);
    double u_dxdx = m_function->Get_y_dxdx(x);
    double tmp = 1 + u * u;

    c_y = std::atan(u);
    c_y_dx = u_dx / tmp;
    c_y_dxdx = u_dxdx / tmp - 2. * u_dx * u_dx * u / (tmp * tmp);

  }

  FrATanFunction atan(const FrFunctionBase &function) {
    return FrATanFunction(function);
  }


}  // end namespace frydom
