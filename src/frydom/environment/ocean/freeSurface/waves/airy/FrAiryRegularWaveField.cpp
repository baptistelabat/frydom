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

    template<class StretchingType>
    FrAiryRegularWaveField<StretchingType>::FrAiryRegularWaveField(FrFreeSurface *freeSurface) :
        FrAiryWaveField<StretchingType>(freeSurface) {
//        m_waveModel = LINEAR_WAVES;
      this->m_verticalFactor = std::make_unique<FrKinematicStretching>();
      this->m_verticalFactor->SetInfDepth(this->m_infinite_depth);
    }

    template<class StretchingType>
    void FrAiryRegularWaveField<StretchingType>::SetWaveHeight(double height) { m_height = height; }

    template<class StretchingType>
    double FrAiryRegularWaveField<StretchingType>::GetWaveHeight() const { return m_height; }

    template<class StretchingType>
    void FrAiryRegularWaveField<StretchingType>::SetWavePeriod(double period) {

      // Set the wave period in seconds
      m_period = period;

      // Set the wave pulsation from the wave period, in rad/s
      m_omega = mathutils::S2RADS(m_period);

      // Set the wave number, using the wave dispersion relation
      auto gravityAcceleration = this->m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();

      if (this->m_infinite_depth) {
        m_k = m_omega * m_omega / gravityAcceleration;
      } else {
        m_k = SolveWaveDispersionRelation(this->m_freeSurface->GetOcean()->GetDepth(NWU), m_omega, gravityAcceleration);
        this->m_infinite_depth = 3. / m_k < this->m_freeSurface->GetOcean()->GetDepth(NWU);
      }
      this->m_verticalFactor->SetInfDepth(this->m_infinite_depth);

    }

    template<class StretchingType>
    double FrAiryRegularWaveField<StretchingType>::GetWavePeriod(FREQUENCY_UNIT unit) const {
      return convert_frequency(m_period, mathutils::S, unit);
    }


    template<class StretchingType>
    void FrAiryRegularWaveField<StretchingType>::SetDirection(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                              DIRECTION_CONVENTION dc) {
      // The wave direction angle is used internally with the convention NWU, GOTO, and RAD unit.
      m_dirAngle = dirAngle;
      if (unit == mathutils::DEG) m_dirAngle *= DEG2RAD;
      if (IsNED(fc)) m_dirAngle = -m_dirAngle;
      if (IsCOMEFROM(dc)) m_dirAngle -= MU_PI;

      mathutils::Normalize_0_2PI(m_dirAngle);

    }

    template<class StretchingType>
    void FrAiryRegularWaveField<StretchingType>::SetDirection(const Direction &direction, FRAME_CONVENTION fc,
                                                              DIRECTION_CONVENTION dc) {
      assert(mathutils::IsClose(direction.Getuz(), 0.));
      double dirAngle = atan2(direction.Getuy(), direction.Getux());
      SetDirection(dirAngle, mathutils::RAD, fc, dc);
    }

    template<class StretchingType>
    double
    FrAiryRegularWaveField<StretchingType>::GetDirectionAngle(ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                              DIRECTION_CONVENTION dc) const {
      double dirAngle = m_dirAngle;
      if (IsNED(fc)) dirAngle = -dirAngle;
      if (IsCOMEFROM(dc)) dirAngle -= MU_PI;
      if (unit == mathutils::DEG) dirAngle *= RAD2DEG;

      return mathutils::Normalize_0_360(dirAngle);
    }

    template<class StretchingType>
    Direction FrAiryRegularWaveField<StretchingType>::GetDirection(FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const {
      auto dirAngle = GetDirectionAngle(mathutils::RAD, fc, dc);
      return {cos(dirAngle), sin(dirAngle), 0.};
    }

    template<class StretchingType>
    double FrAiryRegularWaveField<StretchingType>::GetWaveLength() const { return 2. * M_PI / m_k; }

//    void FrAiryRegularWaveField<StretchingType>::SetStretching(STRETCHING_TYPE type) {
//        switch (type) {
//            case NO_STRETCHING:
//                m_verticalFactor = std::make_unique<FrKinematicStretching>();
//                break;
//            case VERTICAL:
//                m_verticalFactor = std::make_unique<FrKinStretchingVertical>();
//                break;
//            case EXTRAPOLATE:
//                m_verticalFactor = std::make_unique<FrKinStretchingExtrapol>();
//                break;
//            case WHEELER:
//                m_verticalFactor = std::make_unique<FrKinStretchingWheeler>(this);
//                break;
//            case CHAKRABARTI:
//                m_verticalFactor = std::make_unique<FrKinStretchingChakrabarti>(this);
//            case DELTA:
//                m_verticalFactor = std::make_unique<FrKinStretchingDelta>(this);
//            default:
//                m_verticalFactor = std::make_unique<FrKinematicStretching>();
//                break;
//        }
//        m_verticalFactor->SetInfDepth(m_infinite_depth);
//    }

    // ------------------------------------- Wave elevation, velocity and acceleration ----------------------------

    template<class StretchingType>
    std::vector<std::vector<Complex>>
    FrAiryRegularWaveField<StretchingType>::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {
      double NWUsign = 1;
      if (IsNED(fc)) {
        y = -y;
        NWUsign = -NWUsign;
      }
      double kdir = x * cos(m_dirAngle) + y * sin(m_dirAngle);
      Complex cmplxElevation = m_height * exp(JJ * (m_k * kdir - m_omega * this->c_time)) * NWUsign * this->c_ramp;
      return std::vector<std::vector<Complex>>(1, std::vector<Complex>(1, cmplxElevation));
    }

    template<class StretchingType>
    mathutils::Vector3d<Complex>
    FrAiryRegularWaveField<StretchingType>::GetComplexVelocity(double x, double y, double z,
                                                               FRAME_CONVENTION fc) const {
      double NWUsign = 1;
      if (IsNED(fc)) {
        y = -y;
        z = -z;
        NWUsign = -NWUsign;
      }
      auto ComplexElevation = GetComplexElevation(x, y, fc);

      auto Vtemp = m_omega * ComplexElevation[0][0] * this->m_verticalFactor->Eval(x, y, z, m_k, this->c_depth);

      auto Vx = cos(m_dirAngle) * Vtemp * NWUsign;
      auto Vy = sin(m_dirAngle) * Vtemp;
      auto Vz =
          -JJ * m_omega / m_k * ComplexElevation[0][0] * this->m_verticalFactor->EvalDZ(x, y, z, m_k, this->c_depth);

      return {Vx, Vy, Vz};
    }

    template<class StretchingType>
    double FrAiryRegularWaveField<StretchingType>::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
      return std::imag(GetComplexElevation(x, y, fc)[0][0]);
    }

    template<class StretchingType>
    Velocity
    FrAiryRegularWaveField<StretchingType>::GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
      auto cplxVel = GetComplexVelocity(x, y, z, fc);
      return {std::imag(cplxVel.x()), std::imag(cplxVel.y()), std::imag(cplxVel.z())};
    }

    template<class StretchingType>
    Acceleration
    FrAiryRegularWaveField<StretchingType>::GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const {
      auto cplxVel = GetComplexVelocity(x, y, z, fc);
      auto cplxAcc = -JJ * m_omega * cplxVel;
      return {std::imag(cplxAcc.x()), std::imag(cplxAcc.y()), std::imag(cplxAcc.z())};

    }

    // ------------------------------------- Pressure ----------------------------

    template<class StretchingType>
    double
    FrAiryRegularWaveField<StretchingType>::GetPressure(double x, double y, double z, FRAME_CONVENTION fc) const {

      // Get the pressure at the position (x,y,z) for a regular Airy wave field.

      double Pressure = 0;

      double Eta = GetElevation(x, y, fc);
      if (this->m_infinite_depth) { // Infinite water depth.
        Pressure = this->c_density * this->c_gravity * Eta * this->m_verticalFactor->Eval(x, y, z, m_k, this->c_depth);

      } else { // Finite water depth.
        Pressure = this->c_density * this->c_gravity * Eta * this->m_verticalFactor->Eval(x, y, z, m_k, this->c_depth) *
                   std::tanh(m_k * this->c_depth);
      }

      return Pressure;

    }

    // ------------------------------------- Wave characteristics ----------------------------

    template<class StretchingType>
    std::vector<double> FrAiryRegularWaveField<StretchingType>::GetWaveFrequencies(FREQUENCY_UNIT unit) const {
      auto omega = convert_frequency(m_omega, mathutils::RADS, unit);
      return std::vector<double>(1, omega);
    }

    template<class StretchingType>
    std::vector<double> FrAiryRegularWaveField<StretchingType>::GetWaveNumbers() const {
      return std::vector<double>(1, m_k);
    }

    template<class StretchingType>
    std::vector<std::vector<double>> FrAiryRegularWaveField<StretchingType>::GetWaveAmplitudes() const {
      return std::vector<std::vector<double>>(1, std::vector<double>(1, m_height));
    }

    template<class StretchingType>
    std::vector<double> FrAiryRegularWaveField<StretchingType>::GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                                                  DIRECTION_CONVENTION dc) const {
      auto direction = m_dirAngle;

      if (IsNED(fc)) direction = -direction;
      if (dc == COMEFROM) direction += MU_PI;
      mathutils::Normalize_0_2PI(direction);
      if (unit == mathutils::DEG) direction *= MU_180_PI;

      return std::vector<double>(1, direction);
    }

}  // end namespace frydom
