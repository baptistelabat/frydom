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


#include "FrAiryRegularWaveField.h"

#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveDispersionRelation.h"

#include "MathUtils/Angles.h"

namespace frydom {

  FrAiryRegularWaveField::FrAiryRegularWaveField(FrFreeSurface *freeSurface) : FrWaveField(freeSurface) {
    m_waveModel = LINEAR_WAVES;
    m_verticalFactor = std::make_unique<FrKinematicStretching>();
    m_verticalFactor->SetInfDepth(m_infinite_depth);
  }

  void FrAiryRegularWaveField::SetWaveHeight(double height) { m_height = height; }

  double FrAiryRegularWaveField::GetWaveHeight() const { return m_height; }

  void FrAiryRegularWaveField::SetWavePeriod(double period) {

    // Set the wave period in seconds
    m_period = period;

    // Set the wave pulsation from the wave period, in rad/s
    m_omega = mathutils::S2RADS(m_period);

    // Set the wave number, using the wave dispersion relation
    auto gravityAcceleration = m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();

    if (m_infinite_depth) {
      m_k = m_omega * m_omega / gravityAcceleration;
    } else {
      m_k = SolveWaveDispersionRelation(m_freeSurface->GetOcean()->GetDepth(NWU), m_omega, gravityAcceleration);
      m_infinite_depth = 3. / m_k < m_freeSurface->GetOcean()->GetDepth(NWU);
    }
    m_verticalFactor->SetInfDepth(m_infinite_depth);

  }

  double FrAiryRegularWaveField::GetWavePeriod(FREQUENCY_UNIT unit) const {
    return convert_frequency(m_period, mathutils::S, unit);
  }


  void FrAiryRegularWaveField::SetDirection(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                            DIRECTION_CONVENTION dc) {
    // The wave direction angle is used internally with the convention NWU, GOTO, and RAD unit.
    m_dirAngle = dirAngle;
    if (unit == mathutils::DEG) m_dirAngle *= DEG2RAD;
    if (IsNED(fc)) m_dirAngle = -m_dirAngle;
    if (IsCOMEFROM(dc)) m_dirAngle -= MU_PI;

    mathutils::Normalize_0_2PI(m_dirAngle);

  }

  void FrAiryRegularWaveField::SetDirection(const Direction &direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
    assert(mathutils::IsClose(direction.Getuz(), 0.));
    double dirAngle = atan2(direction.Getuy(), direction.Getux());
    SetDirection(dirAngle, mathutils::RAD, fc, dc);
  }

  double
  FrAiryRegularWaveField::GetDirectionAngle(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const {
    double dirAngle = m_dirAngle;
    if (IsNED(fc)) dirAngle = -dirAngle;
    if (IsCOMEFROM(dc)) dirAngle -= MU_PI;
    if (unit == mathutils::DEG) dirAngle *= RAD2DEG;

    return mathutils::Normalize_0_360(dirAngle);
  }

  Direction FrAiryRegularWaveField::GetDirection(FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const {
    auto dirAngle = GetDirectionAngle(mathutils::RAD, fc, dc);
    return {cos(dirAngle), sin(dirAngle), 0.};
  }

  double FrAiryRegularWaveField::GetWaveLength() const { return 2. * M_PI / m_k; }

  void FrAiryRegularWaveField::SetStretching(STRETCHING_TYPE type) {
    switch (type) {
      case NO_STRETCHING:
        m_verticalFactor = std::make_unique<FrKinematicStretching>();
        break;
      case CUTOFF:
        m_verticalFactor = std::make_unique<FrKinStretchingCutoff>();
        break;
      case VERTICAL:
        m_verticalFactor = std::make_unique<FrKinStretchingVertical>();
        break;
      case EXTRAPOLATE:
        m_verticalFactor = std::make_unique<FrKinStretchingExtrapol>();
        break;
      case WHEELER:
        m_verticalFactor = std::make_unique<FrKinStretchingWheeler>(this);
        break;
      case CHAKRABARTI:
        m_verticalFactor = std::make_unique<FrKinStretchingChakrabarti>(this);
      case DELTA:
        m_verticalFactor = std::make_unique<FrKinStretchingDelta>(this);
      default:
        m_verticalFactor = std::make_unique<FrKinematicStretching>();
        break;
    }
    m_verticalFactor->SetInfDepth(m_infinite_depth);
  }

  // ------------------------------------- Wave elevation, velocity and acceleration ----------------------------

  std::vector<std::vector<Complex>>
  FrAiryRegularWaveField::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {
    double NWUsign = 1;
    if (IsNED(fc)) {
      y = -y;
      NWUsign = -NWUsign;
    }
    double kdir = x * cos(m_dirAngle) + y * sin(m_dirAngle);
    Complex cmplxElevation = m_height * exp(JJ * (m_k * kdir - m_omega * c_time)) * NWUsign * c_ramp;
    return std::vector<std::vector<Complex>>(1, std::vector<Complex>(1, cmplxElevation));
  }

  mathutils::Vector3d<Complex>
  FrAiryRegularWaveField::GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
    double NWUsign = 1;
    if (IsNED(fc)) {
      y = -y;
      z = -z;
      NWUsign = -NWUsign;
    }
    auto ComplexElevation = GetComplexElevation(x, y, fc);

    auto Vtemp = m_omega * ComplexElevation[0][0] * m_verticalFactor->Eval(x, y, z, m_k, c_depth);

    auto Vx = cos(m_dirAngle) * Vtemp * NWUsign;
    auto Vy = sin(m_dirAngle) * Vtemp;
    auto Vz = -JJ * m_omega / m_k * ComplexElevation[0][0] * m_verticalFactor->EvalDZ(x, y, z, m_k, c_depth);

    return {Vx, Vy, Vz};
  }

  double FrAiryRegularWaveField::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
    return std::imag(GetComplexElevation(x, y, fc)[0][0]);
  }

  Velocity FrAiryRegularWaveField::GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
    auto cplxVel = GetComplexVelocity(x, y, z, fc);
    return {std::imag(cplxVel.x()), std::imag(cplxVel.y()), std::imag(cplxVel.z())};
  }

  Acceleration FrAiryRegularWaveField::GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const {
    auto cplxVel = GetComplexVelocity(x, y, z, fc);
    auto cplxAcc = -JJ * m_omega * cplxVel;
    return {std::imag(cplxAcc.x()), std::imag(cplxAcc.y()), std::imag(cplxAcc.z())};

  }

  // ------------------------------------- Pressure ----------------------------

  double FrAiryRegularWaveField::GetPressure(double x, double y, double z, FRAME_CONVENTION fc) const {

    // Get the pressure at the position (x,y,z) for a regular Airy wave field.

    double Pressure = 0;

    double Eta = GetElevation(x, y, fc);
    if (m_infinite_depth) { // Infinite water depth.
      Pressure = c_density * c_gravity * Eta * m_verticalFactor->Eval(x, y, z, m_k, c_depth);

    } else { // Finite water depth.
      Pressure = c_density * c_gravity * Eta * m_verticalFactor->Eval(x, y, z, m_k, c_depth) * tanh(m_k * c_depth);
    }

    return Pressure;

  }

  // ------------------------------------- Wave characteristics ----------------------------

  std::vector<double> FrAiryRegularWaveField::GetWaveFrequencies(FREQUENCY_UNIT unit) const {
    auto omega = convert_frequency(m_omega, mathutils::RADS, unit);
    return std::vector<double>(1, omega);
  }

  std::vector<double> FrAiryRegularWaveField::GetWaveNumbers() const {
    return std::vector<double>(1, m_k);
  }

  std::vector<std::vector<double>> FrAiryRegularWaveField::GetWaveAmplitudes() const {
    return std::vector<std::vector<double>>(1, std::vector<double>(1, m_height));
  }

  std::vector<double>
  FrAiryRegularWaveField::GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const {
    auto direction = m_dirAngle;

    if (IsNED(fc)) direction = -direction;
    if (dc == COMEFROM) direction += MU_PI;
    mathutils::Normalize_0_2PI(direction);
    if (unit == mathutils::DEG) direction *= MU_180_PI;

    return std::vector<double>(1, direction);
  }

}  // end namespace frydom
