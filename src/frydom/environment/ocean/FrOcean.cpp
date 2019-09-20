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


namespace frydom {

    template<typename OffshoreSystemType>
    FrOcean<OffshoreSystemType>::FrOcean(FrEnvironment<OffshoreSystemType> *environment) :m_environment(environment) {

      m_seabed = std::make_unique<FrMeanSeabed>(this);
      m_freeSurface = std::make_unique<FrFreeSurface>(this);
      m_current = std::make_unique<FrCurrent>(this);
      m_waterProp = std::make_unique<FrFluidProperties>(10., 1027., 0.001397, 1.3604E-06, 35., 1.2030E-03);

    }

    template<typename OffshoreSystemType>
    FrEnvironment<OffshoreSystemType>* FrOcean<OffshoreSystemType>::GetEnvironment() const { return m_environment; }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::SetTemperature(double Temperature) { m_waterProp->m_temperature = Temperature; }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetTemperature() const { return m_waterProp->m_temperature; }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::SetDensity(double Density) { m_waterProp->m_density = Density; }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetDensity() const { return m_waterProp->m_density; }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::SetDynamicViscosity(double DynamicViscosity) { m_waterProp->m_dynamicViscosity = DynamicViscosity; }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetDynamicViscosity() const { return m_waterProp->m_dynamicViscosity; }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::SetKinematicViscosity(
        double KinematicViscosity) { m_waterProp->m_kinematicViscosity = KinematicViscosity; }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetKinematicViscosity() const { return m_waterProp->m_kinematicViscosity; }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::SetSalinity(double Salinity) { m_waterProp->m_salinity = Salinity; }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetSalinity() const { return m_waterProp->m_salinity; }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::SetPressure(double Pressure) { m_waterProp->m_pressure = Pressure; }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetPressure() const { return m_waterProp->m_pressure; }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetReynoldsNumberInWater(double characteristicLength, double velocity) const {
      return fabs(velocity) * characteristicLength / GetKinematicViscosity();
    }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetFroudeNumberInWater(double characteristicLength, double velocity) const {
      return fabs(velocity) / sqrt(m_environment->GetGravityAcceleration() * characteristicLength);
    }

    template<typename OffshoreSystemType>
    FrFreeSurface<OffshoreSystemType> *FrOcean<OffshoreSystemType>::GetFreeSurface() const { return m_freeSurface.get(); }

    template<typename OffshoreSystemType>
    FrCurrent<OffshoreSystemType> *FrOcean<OffshoreSystemType>::GetCurrent() const { return m_current.get(); }

    template<typename OffshoreSystemType>
    FrSeabed<OffshoreSystemType> *FrOcean<OffshoreSystemType>::GetSeabed() const { return m_seabed.get(); }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::Update(double time) {

      m_freeSurface->Update(time);
      m_current->Update(time);
      m_seabed->Update(time);

    }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::Initialize() {
      m_freeSurface->Initialize();
      m_current->Initialize();
      m_seabed->Initialize();
    }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::StepFinalize() {
      m_freeSurface->StepFinalize();
      m_current->StepFinalize();
      m_seabed->StepFinalize();
    }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetDepth(FRAME_CONVENTION fc) const {
      return m_freeSurface->GetTidal()->GetHeight(fc) - m_seabed->GetBathymetry(fc);
    }

    template<typename OffshoreSystemType>
    double FrOcean<OffshoreSystemType>::GetDepth(double x, double y, FRAME_CONVENTION fc) const {
      return m_freeSurface->GetTidal()->GetHeight(fc) - m_seabed->GetBathymetry(x, y, fc);
    }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::ShowSeabed(bool showSeabed) {
      if (showSeabed) {
//            assert(dynamic_cast<FrNullSeabed>(m_seabed)); //FIXME to check that the deleted seabed is a NullSeabed
        m_seabed = std::make_unique<FrMeanSeabed>(this);
      } else {
        m_seabed = std::make_unique<FrNullSeabed>(this);
      }
    }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::ShowFreeSurface(bool showFreeSurface) {
      m_freeSurface->ShowFreeSurface(showFreeSurface);
    }

    template<typename OffshoreSystemType>
    void FrOcean<OffshoreSystemType>::SetInfiniteDepth() { ShowSeabed(false); }

}  // end namespace frydom
