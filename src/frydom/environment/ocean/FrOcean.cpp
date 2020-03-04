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


#include "FrOcean.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"
#include "frydom/environment/FrFluidType.h"
#include "freeSurface/FrFreeSurface.h"
#include "seabed/FrSeabed.h"
#include "current/FrCurrent.h"
#include "frydom/logging/FrEventLogger.h"


namespace frydom {

  FrOcean::FrOcean(FrEnvironment *environment) : m_environment(environment) {

    m_seabed = std::make_unique<FrFlatSeabed>(this);
    m_freeSurface = std::make_unique<FrFreeSurface>(this);
    m_current = std::make_unique<FrCurrent>(this);
    m_waterProp = std::make_unique<FrFluidProperties>(10., 1027., 0.001397, 1.3604E-06, 35., 1.2030E-03);

  }

  FrEnvironment *FrOcean::GetEnvironment() const { return m_environment; }


  void FrOcean::SetTemperature(double Temperature) { m_waterProp->m_temperature = Temperature; }

  double FrOcean::GetTemperature() const { return m_waterProp->m_temperature; }

  void FrOcean::SetDensity(double Density) { m_waterProp->m_density = Density; }

  double FrOcean::GetDensity() const { return m_waterProp->m_density; }

  void FrOcean::SetDynamicViscosity(double DynamicViscosity) { m_waterProp->m_dynamicViscosity = DynamicViscosity; }

  double FrOcean::GetDynamicViscosity() const { return m_waterProp->m_dynamicViscosity; }

  void
  FrOcean::SetKinematicViscosity(double KinematicViscosity) { m_waterProp->m_kinematicViscosity = KinematicViscosity; }

  double FrOcean::GetKinematicViscosity() const { return m_waterProp->m_kinematicViscosity; }

  void FrOcean::SetSalinity(double Salinity) { m_waterProp->m_salinity = Salinity; }

  double FrOcean::GetSalinity() const { return m_waterProp->m_salinity; }

  void FrOcean::SetPressure(double Pressure) { m_waterProp->m_pressure = Pressure; }

  double FrOcean::GetPressure() const { return m_waterProp->m_pressure; }

  double FrOcean::GetReynoldsNumberInWater(double characteristicLength, double velocity) const {
    return fabs(velocity) * characteristicLength / GetKinematicViscosity();
  }

  double FrOcean::GetFroudeNumberInWater(double characteristicLength, double velocity) const {
    return fabs(velocity) / sqrt(m_environment->GetGravityAcceleration() * characteristicLength);
  }

  FrFreeSurface *FrOcean::GetFreeSurface() const { return m_freeSurface.get(); }

  FrCurrent *FrOcean::GetCurrent() const { return m_current.get(); }

  FrSeabed *FrOcean::GetSeabed() const { return m_seabed.get(); }

  void FrOcean::Update(double time) {

    m_freeSurface->Update(time);
    m_current->Update(time);
    m_seabed->Update(time);

  }

  void FrOcean::Initialize() {
    event_logger::info("Ocean", "", "BEGIN Ocean initialization");
    event_logger::flush();
    m_freeSurface->Initialize();
    m_current->Initialize();
    m_seabed->Initialize();
    event_logger::info("Ocean", "", "END Ocean initialization");
    event_logger::flush();
  }

  void FrOcean::StepFinalize() {
    m_freeSurface->StepFinalize();
    m_current->StepFinalize();
    m_seabed->StepFinalize();
  }

  double FrOcean::GetDepth(FRAME_CONVENTION fc) const {
    return m_freeSurface->GetTidal()->GetHeight(fc) - m_seabed->GetBathymetry(fc);
  }

  double FrOcean::GetDepth(double x, double y, FRAME_CONVENTION fc) const {
    return m_freeSurface->GetTidal()->GetHeight(fc) - m_seabed->GetBathymetry(x, y, fc);
  }

  void FrOcean::ShowSeabed(bool showSeabed) {
    if (!showSeabed) {
      m_seabed->DontShow();
    }
  }

  void FrOcean::ShowFreeSurface(bool showFreeSurface) {
    m_freeSurface->Show(showFreeSurface);
  }

  void FrOcean::SetInfiniteDepth() { ShowSeabed(false); }

}  // end namespace frydom
