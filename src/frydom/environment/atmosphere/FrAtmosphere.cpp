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

    template<typename OffshoreSystemType>
    FrAtmosphere<OffshoreSystemType>::FrAtmosphere(FrEnvironment<OffshoreSystemType> *environment) {

      m_environment = environment;

      m_wind = std::make_unique<FrWind>(this);
      m_airProp = std::make_unique<FrFluidProperties>(20., 1.204, 0., 0., 0., 0.);

    }

    template<typename OffshoreSystemType>
    FrEnvironment<OffshoreSystemType> *FrAtmosphere<OffshoreSystemType>::GetEnvironment() const { return m_environment; }

    template<typename OffshoreSystemType>
    void FrAtmosphere<OffshoreSystemType>::SetTemperature(double Temperature) { m_airProp->m_temperature = Temperature; }

    template<typename OffshoreSystemType>
    double FrAtmosphere<OffshoreSystemType>::GetTemperature() const { return m_airProp->m_temperature; }

    template<typename OffshoreSystemType>
    void FrAtmosphere<OffshoreSystemType>::SetDensity(double Density) { m_airProp->m_density = Density; }

    template<typename OffshoreSystemType>
    double FrAtmosphere<OffshoreSystemType>::GetDensity() const { return m_airProp->m_density; }

    template<typename OffshoreSystemType>
    void
    FrAtmosphere<OffshoreSystemType>::SetDynamicViscosity(double DynamicViscosity) { m_airProp->m_dynamicViscosity = DynamicViscosity; }

    template<typename OffshoreSystemType>
    double FrAtmosphere<OffshoreSystemType>::GetDynamicViscosity() const { return m_airProp->m_dynamicViscosity; }

    template<typename OffshoreSystemType>
    void FrAtmosphere<OffshoreSystemType>::SetKinematicViscosity(
        double KinematicViscosity) { m_airProp->m_kinematicViscosity = KinematicViscosity; }

    template<typename OffshoreSystemType>
    double FrAtmosphere<OffshoreSystemType>::GetKinematicViscosity() const { return m_airProp->m_kinematicViscosity; }

    template<typename OffshoreSystemType>
    void FrAtmosphere<OffshoreSystemType>::SetSalinity(double Salinity) { m_airProp->m_salinity = Salinity; }

    template<typename OffshoreSystemType>
    double FrAtmosphere<OffshoreSystemType>::GetSalinity() const { return m_airProp->m_salinity; }

    template<typename OffshoreSystemType>
    void FrAtmosphere<OffshoreSystemType>::SetPressure(double Pressure) { m_airProp->m_pressure = Pressure; }

    template<typename OffshoreSystemType>
    double FrAtmosphere<OffshoreSystemType>::GetPressure() const { return m_airProp->m_pressure; }

    template<typename OffshoreSystemType>
    double FrAtmosphere<OffshoreSystemType>::GetReynoldsNumberInAir(double characteristicLength, double velocity) const {
      return fabs(velocity) * characteristicLength / GetKinematicViscosity();
    }

    template<typename OffshoreSystemType>
    double FrAtmosphere<OffshoreSystemType>::GetFroudeNumberInAir(double characteristicLength, double velocity) const {
      return fabs(velocity) / std::sqrt(m_environment->GetGravityAcceleration() * characteristicLength);
    }

    template<typename OffshoreSystemType>
    FrWind<OffshoreSystemType> *FrAtmosphere<OffshoreSystemType>::GetWind() const { return m_wind.get(); }

    template<typename OffshoreSystemType>
    void FrAtmosphere<OffshoreSystemType>::Update(double time) {
      m_wind->Update(time);
    }

    template<typename OffshoreSystemType>
    void FrAtmosphere<OffshoreSystemType>::Initialize() {
      m_wind->Initialize();
    }

    template<typename OffshoreSystemType>
    void FrAtmosphere<OffshoreSystemType>::StepFinalize() {
      m_wind->StepFinalize();
    }


}  // end namespace frydom
