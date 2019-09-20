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


#include <frydom/core/math/functions/ramp/FrCosRampFunction.h>
#include "FrEnvironment.h"

#include "frydom/core/math/functions/ramp/FrLinearRampFunction.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "time/FrTimeServices.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "ocean/freeSurface/FrFreeSurface.h"
#include "ocean/seabed/FrSeabed.h"
#include "ocean/current/FrCurrent.h"
#include "frydom/environment/atmosphere/FrAtmosphere.h"
#include "atmosphere/wind/FrWind.h"
#include "frydom/environment/geographicServices/FrGeographicServices.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrEnvironment<OffshoreSystemType>::FrEnvironment(FrOffshoreSystem<OffshoreSystemType> *system) {

      m_system = system;

      m_geographicServices = std::make_unique<FrGeographicServices>();
      m_timeServices = std::make_unique<FrTimeServices>();
      m_ocean = std::make_unique<FrOcean>(this);
      m_atmosphere = std::make_unique<FrAtmosphere>(this);

      m_timeRamp = std::make_unique<FrCosRampFunction>();
      m_timeRamp->SetActive(false);
      m_timeRamp->SetByTwoPoints(0., 0., 10., 1.);

//        if (not(m_infinite_depth)) m_seabed->SetEnvironment(this); // TODO : voir a porter ca dans seabed...

      SetGravityAcceleration(9.81);
    }

    template<typename OffshoreSystemType>
    FrEnvironment<OffshoreSystemType>::~FrEnvironment() = default;

    template<typename OffshoreSystemType>
    FrOffshoreSystem<OffshoreSystemType> *FrEnvironment<OffshoreSystemType>::GetSystem() { return m_system; }

    template<typename OffshoreSystemType>
    double FrEnvironment<OffshoreSystemType>::GetTime() const { return m_system->GetTime(); } // TODO : voir a gerer l'UTC etc...
    template<typename OffshoreSystemType>
    double FrEnvironment<OffshoreSystemType>::GetGravityAcceleration() const {
      return m_system->GetGravityAcceleration();
    }

    template<typename OffshoreSystemType>
    void FrEnvironment<OffshoreSystemType>::SetGravityAcceleration(double gravityAcceleration) {
      m_system->SetGravityAcceleration(gravityAcceleration);
    }

    template<typename OffshoreSystemType>
    FrOcean<OffshoreSystemType> *FrEnvironment<OffshoreSystemType>::GetOcean() const { return m_ocean.get(); }

    template<typename OffshoreSystemType>
    FrAtmosphere<OffshoreSystemType> *FrEnvironment<OffshoreSystemType>::GetAtmosphere() const { return m_atmosphere.get(); }

    template<typename OffshoreSystemType>
    Velocity FrEnvironment<OffshoreSystemType>::GetRelativeVelocityInFrame(const FrFrame &frame, const Velocity &worldVel,
                                                       FLUID_TYPE ft, FRAME_CONVENTION fc) {
      switch (ft) {
        case WATER:
          return m_ocean->GetCurrent()->GetRelativeVelocityInFrame(frame, worldVel, fc);
        case AIR:
          return m_atmosphere->GetWind()->GetRelativeVelocityInFrame(frame, worldVel, fc);
        default:
          throw FrException("Fluid is not known...");
      }
    }

    template<typename OffshoreSystemType>
    double FrEnvironment<OffshoreSystemType>::GetFluidDensity(FLUID_TYPE ft) const {
      switch (ft) {
        case AIR:
          return m_atmosphere->GetDensity();
        case WATER:
          return m_ocean->GetDensity();
      }
    }

    template<typename OffshoreSystemType>
    FrGeographicServices *FrEnvironment<OffshoreSystemType>::GetGeographicServices() const {
      return m_geographicServices.get();
    }

    template<typename OffshoreSystemType>
    int FrEnvironment<OffshoreSystemType>::GetYear() const {
      /// Get the UTC time to obtain the year
      auto lt = GetTimeServices()->GetUTCTime();
      date::year_month_day ymd{date::floor<date::days>(lt)};
      return int(ymd.year());
    }

    template<typename OffshoreSystemType>
    void FrEnvironment<OffshoreSystemType>::ShowFreeSurface(bool show) {
      GetOcean()->GetFreeSurface()->ShowFreeSurface(show);
    }

    template<typename OffshoreSystemType>
    void FrEnvironment<OffshoreSystemType>::ShowSeabed(bool show) {
      GetOcean()->ShowSeabed(show);
    }

    template<typename OffshoreSystemType>
    FrTimeServices *FrEnvironment<OffshoreSystemType>::GetTimeServices() const { return m_timeServices.get(); }

    template<typename OffshoreSystemType>
    void FrEnvironment<OffshoreSystemType>::Update(double time) {
//        m_timeRamp->Update(time);
      m_ocean->Update(time);
      m_atmosphere->Update(time);
      m_timeServices->Update(time);
    }

    template<typename OffshoreSystemType>
    void FrEnvironment<OffshoreSystemType>::Initialize() {
//      m_timeRamp->Initialize();

      m_ocean->Initialize();
      m_atmosphere->Initialize();
      m_timeServices->Initialize();
    }

    template<typename OffshoreSystemType>
    void FrEnvironment<OffshoreSystemType>::StepFinalize() {
//      m_timeRamp->StepFinalize();
      m_ocean->StepFinalize();
      m_atmosphere->StepFinalize();
    }

    template<typename OffshoreSystemType>
    FrCosRampFunction *FrEnvironment<OffshoreSystemType>::GetTimeRamp() const {
      return m_timeRamp.get();
    }

}  // end namespace frydom
