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


#include "FrAtmosphere.h"

#include "wind/FrWind.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/FrFluidType.h"


namespace frydom {

  FrAtmosphere::FrAtmosphere(FrEnvironment *environment) {

    m_environment = environment;

    m_wind = std::make_unique<FrWind>(this);
    m_airProp = std::make_unique<FrFluidProperties>(20., 1.204, 0., 0., 0., 0.);

  }

  FrEnvironment *FrAtmosphere::GetEnvironment() const { return m_environment; }

  void FrAtmosphere::SetTemperature(double Temperature) { m_airProp->m_temperature = Temperature; }

  double FrAtmosphere::GetTemperature() const { return m_airProp->m_temperature; }

  void FrAtmosphere::SetDensity(double Density) { m_airProp->m_density = Density; }

  double FrAtmosphere::GetDensity() const { return m_airProp->m_density; }

  void FrAtmosphere::SetDynamicViscosity(double DynamicViscosity) { m_airProp->m_dynamicViscosity = DynamicViscosity; }

  double FrAtmosphere::GetDynamicViscosity() const { return m_airProp->m_dynamicViscosity; }

  void FrAtmosphere::SetKinematicViscosity(
      double KinematicViscosity) { m_airProp->m_kinematicViscosity = KinematicViscosity; }

  double FrAtmosphere::GetKinematicViscosity() const { return m_airProp->m_kinematicViscosity; }

  void FrAtmosphere::SetSalinity(double Salinity) { m_airProp->m_salinity = Salinity; }

  double FrAtmosphere::GetSalinity() const { return m_airProp->m_salinity; }

  void FrAtmosphere::SetPressure(double Pressure) { m_airProp->m_pressure = Pressure; }

  double FrAtmosphere::GetPressure() const { return m_airProp->m_pressure; }

  double FrAtmosphere::GetReynoldsNumberInAir(double characteristicLength, double velocity) const {
    return fabs(velocity) * characteristicLength / GetKinematicViscosity();
  }

  double FrAtmosphere::GetFroudeNumberInAir(double characteristicLength, double velocity) const {
    return fabs(velocity) / std::sqrt(m_environment->GetGravityAcceleration() * characteristicLength);
  }

  FrWind *FrAtmosphere::GetWind() const { return m_wind.get(); }

  void FrAtmosphere::Update(double time) {
    m_wind->Update(time);
  }

  void FrAtmosphere::Initialize() {
    m_wind->Initialize();
  }

  void FrAtmosphere::StepFinalize() {
    m_wind->StepFinalize();
  }


}  // end namespace frydom
