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


#include "FrAtmosphere_.h"

//#include "frydom/environment/FrEnvironment.h"
//#include "wind/FrWind.h"


namespace frydom{

    FrAtmosphere_::FrAtmosphere_(FrEnvironment_* environment) {

        m_environment = environment;

        m_wind       = std::make_unique<FrWind_>(this);
        m_airProp     = std::make_unique<FrFluidProperties>(20., 1.204, 0., 0., 0., 0. );

    }

    FrEnvironment_ *FrAtmosphere_::GetEnvironment() const { return m_environment; }

    void FrAtmosphere_::SetTemperature(double Temperature) {m_airProp->m_temperature = Temperature;}

    double FrAtmosphere_::GetTemperature() const {return m_airProp->m_temperature;}

    void FrAtmosphere_::SetDensity(double Density) {m_airProp->m_density = Density;}

    double FrAtmosphere_::GetDensity() const {return m_airProp->m_density;}

    void FrAtmosphere_::SetDynamicViscosity(double DynamicViscosity) {m_airProp->m_dynamicViscosity = DynamicViscosity;}

    double FrAtmosphere_::GetDynamicViscosity() const {return m_airProp->m_dynamicViscosity;}

    void FrAtmosphere_::SetKinematicViscosity(double KinematicViscosity) {m_airProp->m_kinematicViscosity = KinematicViscosity;}

    double FrAtmosphere_::GetKinematicViscosity() const {return m_airProp->m_kinematicViscosity;}

    void FrAtmosphere_::SetSalinity(double Salinity) {m_airProp->m_salinity = Salinity;}

    double FrAtmosphere_::GetSalinity() const {return m_airProp->m_salinity;}

    void FrAtmosphere_::SetPressure(double Pressure) {m_airProp->m_pressure = Pressure;}

    double FrAtmosphere_::GetPressure() const {return m_airProp->m_pressure;}

    double FrAtmosphere_::GetReynoldsNumberInAir(double characteristicLength, double velocity) const {
        return fabs(velocity) * characteristicLength / GetKinematicViscosity();
    }

    double FrAtmosphere_::GetFroudeNumberInAir(double characteristicLength, double velocity) const {
        return fabs(velocity) / sqrt(m_environment->GetGravityAcceleration() * characteristicLength);
    }

    FrWind_ *FrAtmosphere_::GetWind() const { return m_wind.get();}

    void FrAtmosphere_::Update(double time) {
        m_wind->Update(time);
    }

    void FrAtmosphere_::Initialize() {
        m_wind->Initialize();
    }

    void FrAtmosphere_::StepFinalize() {
        m_wind->StepFinalize();
    }


}  // end namespace frydom
