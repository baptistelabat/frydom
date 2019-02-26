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


#include "FrEnvironment.h"

#include "frydom/core/math/functions/ramp/FrLinearRampFunction.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "time/FrTimeZone.h"
#include "ocean/FrOcean_.h"
#include "ocean/freeSurface/FrFreeSurface.h"
#include "ocean/seabed/FrSeabed.h"
#include "ocean/current/FrCurrent.h"
#include "atmosphere/FrAtmosphere_.h"
#include "atmosphere/wind/FrWind.h"
#include "frydom/environment/geographicServices/FrGeographicServices.h"


namespace frydom {

    FrEnvironment_::FrEnvironment_(FrOffshoreSystem_* system) {

        m_system = system;

        m_geographicServices    = std::make_unique<FrGeographicServices>();
        m_timeZone              = std::make_unique<FrTimeZone>();
        m_ocean                 = std::make_unique<FrOcean_>(this);
        m_atmosphere            = std::make_unique<FrAtmosphere_>(this);

        m_timeRamp              = std::make_unique<FrLinearRampFunction_>();
        m_timeRamp->SetActive(false);

//        if (not(m_infinite_depth)) m_seabed->SetEnvironment(this); // TODO : voir a porter ca dans seabed...

        SetGravityAcceleration(9.81);
    }

    FrEnvironment_::~FrEnvironment_() = default;

    FrOffshoreSystem_* FrEnvironment_::GetSystem() { return m_system; }

    double FrEnvironment_::GetTime() const { return m_system->GetTime(); } // TODO : voir a gerer l'UTC etc...


    double FrEnvironment_::GetGravityAcceleration() const {
        return m_system->GetGravityAcceleration();
    }

    void FrEnvironment_::SetGravityAcceleration(double gravityAcceleration) {
        m_system->SetGravityAcceleration(gravityAcceleration);
    }


    FrOcean_ *FrEnvironment_::GetOcean() const { return m_ocean.get();}

    FrAtmosphere_ *FrEnvironment_::GetAtmosphere() const { return m_atmosphere.get();}

    Velocity FrEnvironment_::GetRelativeVelocityInFrame(const FrFrame_& frame, const Velocity& worldVel,
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

    double FrEnvironment_::GetFluidDensity(FLUID_TYPE ft) const {
        switch (ft) {
            case AIR:
                return m_atmosphere->GetDensity();
            case WATER:
                return m_ocean->GetDensity();
        }
    }

    FrGeographicServices *FrEnvironment_::GetGeographicServices() const {
        return m_geographicServices.get();
    }

    int FrEnvironment_::GetYear() const {
        /// Get the UTC time to obtain the year
        auto lt = GetTimeZone()->GetUTCTime();
        date::year_month_day ymd{date::floor<date::days>(lt)};
        return int(ymd.year());
    }

    void FrEnvironment_::ShowFreeSurface(bool show) {
        GetOcean()->GetFreeSurface()->ShowFreeSurface(show);
    }

    void FrEnvironment_::ShowSeabed(bool show) {
//        GetOcean()->GetSeabed()->
        // TODO
    }

    FrTimeZone *FrEnvironment_::GetTimeZone() const {return m_timeZone.get();}

    void FrEnvironment_::Update(double time) {
//        m_timeRamp->Update(time);
        m_ocean->Update(time);
        m_atmosphere->Update(time);
        m_timeZone->Update(time);
    }

    void FrEnvironment_::Initialize() {
        m_timeRamp->Initialize();

        m_ocean->Initialize();
        m_atmosphere->Initialize();
        m_timeZone->Initialize();
    }

    void FrEnvironment_::StepFinalize() {
        m_timeRamp->StepFinalize();
        m_ocean->StepFinalize();
        m_atmosphere->StepFinalize();
    }

    FrLinearRampFunction_ *FrEnvironment_::GetTimeRamp() const { return m_timeRamp.get();}


}  // end namespace frydom
