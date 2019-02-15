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

#include "GeographicLib/LocalCartesian.hpp"
#include "GeographicLib/MagneticModel.hpp"
#include "frydom/core/common/FrException.h"

#include "frydom/core/math/functions/ramp/FrLinearRampFunction.h"

#include "frydom/core/FrOffshoreSystem.h"


#include "time/FrTimeZone.h"

#include "ocean/FrOcean_.h"
#include "ocean/freeSurface/FrFreeSurface.h"
#include "ocean/seabed/FrSeabed.h"
#include "ocean/current/FrCurrent.h"

#include "atmosphere/FrAtmosphere_.h"
#include "atmosphere/wind/FrWind.h"

#include "flow/FrFlowBase.h"


namespace frydom {


    FrEnvironment::FrEnvironment() {

        m_freeSurface = std::make_unique<FrFreeSurface>();
//        m_current = std::make_unique<FrUniformCurrent>();
//        m_wind = std::make_unique<FrUniformWind>();
        m_seabed = std::make_unique<FrSeabed>();
        if (not(m_infinite_depth)) m_seabed->SetEnvironment(this);
        if (m_showSeabed) m_seabed->SetEnvironment(this);
        m_localCartesian = std::make_unique<GeographicLib::LocalCartesian>();
        m_timeZone = std::make_unique<FrTimeZone>();
    }

    FrEnvironment::~FrEnvironment() = default;

    void FrEnvironment::SetSystem(FrOffshoreSystem *system) {
        m_system = system;
    }

    FrOffshoreSystem *FrEnvironment::GetSystem() { return m_system; }

    void FrEnvironment::SetInfiniteDepth(bool is_infinite) {m_infinite_depth = is_infinite;}

    bool FrEnvironment::GetInfiniteDepth() { return m_infinite_depth;}

    void FrEnvironment::SetTime(double time) { m_time = time; }

    double FrEnvironment::GetTime() const { return m_time; }

    double FrEnvironment::GetWaterDensity() const {
        return m_waterDensity;
    }

    void FrEnvironment::SetWaterDensity(const double waterDensity) {
        m_waterDensity = waterDensity;
    }

    double FrEnvironment::GetAirDensity() const {
        return m_airDensity;
    }

    void FrEnvironment::SetAirDensity(double airDensity) {
        m_airDensity = m_airDensity;
    }

    double FrEnvironment::GetGravityAcceleration() const {
        return m_gravityAcceleration;
    }

    void FrEnvironment::SetGravityAcceleration(double gravityAcceleration) {
        m_gravityAcceleration = gravityAcceleration;
    }

    double FrEnvironment::GetSeaTemperature() const {
        return m_seaTemperature;
    }

    void FrEnvironment::SetSeaTemperature(double seaTemperature) {
        m_seaTemperature = m_seaTemperature;
    }

    double FrEnvironment::GetAirtemperature() const {
        return m_airtemperature;
    }

    void FrEnvironment::SetAirtemperature(double airtemperature) {
        m_airtemperature = m_airtemperature;
    }

    double FrEnvironment::GetWaterKinematicViscosity() const {
        // TODO: gerer la temperature
        return m_waterKinematicViscosity;
    }

    void FrEnvironment::SetWaterKinematicViscosity(double waterKinematicViscosity) {
        m_waterKinematicViscosity = m_waterKinematicViscosity;
    }

    double FrEnvironment::GetAtmosphericPressure() const {
        return m_atmosphericPressure;
    }

    void FrEnvironment::SetAtmosphericPressure(double atmosphericPressure) {
        m_atmosphericPressure = m_atmosphericPressure;
    }

    FrFreeSurface *FrEnvironment::GetFreeSurface() const {
        return m_freeSurface.get();
    }

    FrTidal *FrEnvironment::GetTidal() const {
        return m_freeSurface->GetTidal();
    }

//    template<class T>
//    T *FrEnvironment::GetCurrent() const { return dynamic_cast<T*>(m_current.get()); }
//
//    template<class T>
//    T *FrEnvironment::GetWind() const { return dynamic_cast<T*>(m_wind.get()); }

//    void FrEnvironment::SetCurrent(const FrCurrent::MODEL type) {
//
//        switch (type) {
//            case FrCurrent::UNIFORM:
//                m_current = std::make_shared<FrUniformCurrent>();
//                break;
//            default:
//                break;
//        }
//    }
//
//    void FrEnvironment::SetWind(const FrWind::MODEL type) {
//
//        switch (type) {
//            case FrWind::UNIFORM:
//                m_wind = std::make_shared<FrUniformWind>();
//                break;
//            default:
//                break;
//        }
//    }

    FrSeabed *FrEnvironment::GetSeabed() const {
        return m_seabed.get();
    }

    void FrEnvironment::SetSeabed(FrSeabed *seabed) {
        m_seabed = std::unique_ptr<FrSeabed>(seabed);
    }

    GeographicLib::LocalCartesian *FrEnvironment::GetGeoLib() const {
        return m_localCartesian.get();
    }

    void FrEnvironment::SetGeographicOrigin(double lat0, double lon0, double h0) {
        m_localCartesian->Reset(lat0, lon0, h0);
    }

    void FrEnvironment::Convert_GeoToCart(double lat, double lon, double h, double &x, double &y, double &z) {
        m_localCartesian->Forward(lat, lon, h, x, y, z);
    }

    void FrEnvironment::Convert_CartToGeo(double x, double y, double z, double &lat, double &lon, double &h) {
        m_localCartesian->Reverse(x, y, z, lat, lon, h);
    }

    FrTimeZone *FrEnvironment::GetTimeZone() const {return m_timeZone.get();}

    void FrEnvironment::Update(double time) {
        if (m_showFreeSurface)  m_freeSurface->Update(time);
        //m_current->Update(time);
        //m_wind->Update(time);
        if (m_showSeabed) m_seabed->Update(time);
        if (not(m_infinite_depth)) m_seabed->Update(time);
        m_time = time;
        m_timeZone->Update(time);
    }

    void FrEnvironment::Initialize() {
        // TODO: appeler les methodes Initialize() sur les attributs
        if (m_showFreeSurface) m_freeSurface->Initialize();
        //m_current->Initialize();
        //m_wind->Initialize();
        if (m_showSeabed) m_seabed->Initialize();
        if (not(m_infinite_depth)) m_seabed->Initialize();
        m_timeZone->Initialize();
    }

    void FrEnvironment::StepFinalize() {
        if (m_showFreeSurface) m_freeSurface->StepFinalize();
        //m_current->StepFinalize();
        //m_wind->StepFinalize();
        if (m_showSeabed) m_seabed->StepFinalize();
        if (not(m_infinite_depth)) m_seabed->StepFinalize();
    }

//    void FrEnvironment::SetCurrent(FrCurrent *current) {
//        m_current = std::shared_ptr<FrCurrent>(current);
//    }

//    void FrEnvironment::SetFreeSurface(FrFreeSurface *freeSurface) {
//        m_freeSurface = std::unique_ptr<FrFreeSurface>(freeSurface);
//        m_system->AddBody(m_freeSurface->GetBody());
//    }

    int FrEnvironment::GetYear() const {
        /// Get the UTC time to obtain the year
        auto lt = GetTimeZone()->GetUTCTime();
        date::year_month_day ymd{date::floor<date::days>(lt)};
        return int(ymd.year());
    }

    double FrEnvironment::ComputeMagneticDeclination(double x, double y, double z) {

        /// Magnetic model loaded from _deps directory
        GeographicLib::MagneticModel magneticModel("emm2017", "../_deps/magneticmodel-src");
        double lat, lon, h;

        /// Convert the node local coordinates to geographical coordinates
        Convert_CartToGeo(x, y, z, lat, lon, h);

        /// Compute the magnetic declination
        double Bx, By, Bz, H, F, D, I;
        magneticModel(GetYear(), lat, lon, h, Bx, By, Bz);
        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);

        return D;
    }



















    // REFACTORING ---------->>>>>>>>>>

    FrEnvironment_::FrEnvironment_(FrOffshoreSystem_* system) {

        m_system = system;

        m_geographicServices    = std::make_unique<FrGeographicServices>();
        m_timeZone              = std::make_unique<FrTimeZone>();
        m_ocean                 = std::make_unique<FrOcean_>(this);
        m_atmosphere            = std::make_unique<FrAtmosphere_>(this);

        m_timeRamp              = std::make_unique<FrLinearRampFunction_>();

//        if (not(m_infinite_depth)) m_seabed->SetEnvironment(this); // TODO : voir a porter ca dans seabed...

        SetGravityAcceleration(9.81);
    }

    FrEnvironment_::~FrEnvironment_() = default;

    FrOffshoreSystem_* FrEnvironment_::GetSystem() { return m_system; }

//    void FrEnvironment::SetInfiniteDepth(bool is_infinite) {m_infinite_depth = is_infinite;}
//
//    bool FrEnvironment::GetInfiniteDepth() { return m_infinite_depth;}

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
        m_timeRamp->SetActive(false);

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
