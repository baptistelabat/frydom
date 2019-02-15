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


#ifndef FRYDOM_FRENVIRONMENT_H
#define FRYDOM_FRENVIRONMENT_H


#include "frydom/core/common/FrObject.h"

//#include <frydom/environment/wind/FrWind.h>


// Time includes
//#include "FrTimeZone.h"

// Current includes
//#include "current/FrCurrent.h"
//#include "current/FrCurrentPolarCoeffs.h"
//#include "current/FrCurrentForce.h"
//#include "current/FrCurrentStandardForce.h"

// Waves includes
//#include "waves/FrFreeSurface.h"
//#include "waves/FrWaveSpectrum.h"
//#include "waves/FrWaveField.h"
//#include "tidal/FrTidalModel.h"

// Wind includes
//#include "wind/FrWind.h"
//#include "wind/FrWindForce.h"
//#include "wind/FrWindStandardForce.h"

// Seabed includes
//#include "seabed/FrSeabed.h"

// GeographicLib includes
#include "frydom/environment/geographicServices/FrGeographicServices.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/environment/FrFluidType.h"

namespace GeographicLib {
    class LocalCartesian;
}

namespace frydom {

    // Forward declarations
    class FrOffshoreSystem;
    class FrTimeZone;
    class FrFreeSurface;
    class FrTidal;
    class FrCurrent;
//    class FrUniformCurrent;
    class FrWind;
//    class FrUniformWind;

    class FrSeabed;


    /// Class to store the different elements composing the offshore environment

    /**
     * \class FrEnvironment
     * \brief Class for defining the environmental data.
     */
    class FrEnvironment : public FrObject {

    private:

        FrOffshoreSystem* m_system;

        double m_time;
        std::unique_ptr<FrTimeZone> m_timeZone;

        // Environment components
        std::unique_ptr<FrFreeSurface> m_freeSurface;
        std::unique_ptr<FrCurrent> m_current;
        std::unique_ptr<FrWind> m_wind;
        std::unique_ptr<FrSeabed> m_seabed;

        /// Wrap for GeographicLib, contains the origin of geographic coordinates
        std::unique_ptr<GeographicLib::LocalCartesian> m_localCartesian;

        // Environments scalars
        double m_waterDensity = 1025.;
        double m_airDensity = 1.204;

        double m_gravityAcceleration = 9.81;

        double m_seaTemperature = 15.;
        double m_airtemperature = 20.;

        double m_waterKinematicViscosity;

        double m_atmosphericPressure;

        bool m_infinite_depth = false;

        bool m_showSeabed = true;
        bool m_showFreeSurface = true;

    public:

        FrEnvironment();

        ~FrEnvironment();

        void SetSystem(FrOffshoreSystem* system);

        FrOffshoreSystem* GetSystem();

        void SetInfiniteDepth(bool is_infinite);
        bool GetInfiniteDepth();

        void SetShowSeabed(bool is_shown) {m_showSeabed = is_shown;}
        bool GetShowSeabed() { return m_showSeabed;}

        void SetShowFreeSurface(bool is_shown) {m_showFreeSurface = is_shown;}
        bool GetShowFreeSurface() { return m_showFreeSurface;}

        void SetTime(double time);

        double GetTime() const;

        double GetWaterDensity() const;

        void SetWaterDensity(const double waterDensity);

        double GetAirDensity() const;

        void SetAirDensity(double airDensity);

        double GetGravityAcceleration() const;

        void SetGravityAcceleration(double gravityAcceleration);

        double GetSeaTemperature() const;

        void SetSeaTemperature(double seaTemperature);

        double GetAirtemperature() const;

        void SetAirtemperature(double airtemperature);

        double GetWaterKinematicViscosity() const;

        void SetWaterKinematicViscosity(double waterKinematicViscosity);

        double GetAtmosphericPressure() const;

        void SetAtmosphericPressure(double atmosphericPressure);

        FrFreeSurface* GetFreeSurface() const;

//        void SetFreeSurface(FrFreeSurface* freeSurface);

        FrTidal* GetTidal() const;
//
//        void SetTidal(FrTidal* tidal) {
//            m_tidal = std::unique_ptr<FrTidal>(tidal);
//        }

        template <class T=FrCurrent>
        T* GetCurrent() const;

        template <class T=FrWind>
        T* GetWind() const;

//        void SetCurrent(FrCurrent* current);

//        void SetCurrent (const FrCurrent::MODEL type=FrCurrent::UNIFORM);
//
//        void SetWind(const FrWind::MODEL type=FrWind::UNIFORM);

        inline FrSeabed* GetSeabed() const;

        void SetSeabed(FrSeabed* seabed);

        // TODO : encapsuler ces methodes dans un GeographicService

        GeographicLib::LocalCartesian* GetGeoLib() const;

        void SetGeographicOrigin(double lat0, double lon0, double h0);

        void Convert_GeoToCart(double lat, double lon, double h, double& x, double& y, double& z);

        void Convert_CartToGeo(double x, double y, double z, double& lat, double& lon, double& h);

        double ComputeMagneticDeclination(double x, double y, double z);

        /* TODO : DEBUG memory leak due to ptr to m_geoServices
        FrGeographicServices* GetGeoServices() const {
            return m_geoServices.get();
        }

        void SetGeographicOrigin(double lat0, double lon0, double h0){
            m_geoServices->SetGeographicOrigin(lat0,lon0,h0);
        }

        void Convert_GeoToCart(double lat, double lon, double h, double& x, double& y, double& z){
            m_geoServices->GeoToCart(lat,lon,h,x,y,z);
        }

        void Convert_CartToGeo(double x, double y, double z, double& lat, double& lon, double& h){
            m_geoServices->CartToGeo(x,y,z,lat,lon,h);
        }

        double ComputeMagneticDeclination(double x, double y, double z){
            m_geoServices->GetDeclinationFromCart(x,y,z,GetYear());
        }*/

        FrTimeZone* GetTimeZone() const;
        //void SetTimeZoneName(FrTimeZone* TimeZone) {m_timeZoneName = TimeZone;}

        int GetYear() const;  // FIXME : ne devrait pas etre direct dans environment mais dans un servixce de temps...

        void Update(double time);

        void Initialize() override;

        void StepFinalize() override;

    };

    template<class T>
    T* FrEnvironment::GetCurrent() const { return dynamic_cast<T*>(m_current.get()); }

    template<class T>
    T* FrEnvironment::GetWind() const { return dynamic_cast<T*>(m_wind.get()); }














    // REFACTORING ---->>>>







    class FrOffshoreSystem_;

    class FrOcean_;
    class FrAtmosphere_;
    class FrGeographicServices;

    class Velocity;
    class FrFrame_;

    class FrLinearRampFunction_;


    /// Class to store the different elements composing the offshore environment

    /**
     * \class FrEnvironment
     * \brief Class for defining the environmental data.
     */
    class FrEnvironment_ : public FrObject {

    private:

        FrOffshoreSystem_* m_system;    ///< Offshore sytem containing this Environment

        //---------------------------- Environment elements ----------------------------//
        // TODO : faire un service de temps, NEED REFACTO
        std::unique_ptr<FrTimeZone> m_timeZone;     ///< Zoned time conversion service, can give time during simulation in a specified time zone.


        std::unique_ptr<FrLinearRampFunction_> m_timeRamp;        ///< Time ramp, can be applied on wave field, current field, wind field, etc.


        std::unique_ptr<FrOcean_> m_ocean;    ///> Ocean element of the simulation, contains free surface and seabed, current model, water properties, etc.


        std::unique_ptr<FrAtmosphere_> m_atmosphere;    ///> Atmosphere element of the simulation, contains wind model, air properties.


        std::unique_ptr<FrGeographicServices> m_geographicServices;    ///> Service converting local coordinates to geographic coordinates, contains the geocoord origins.

    public:

        /// Default constructor
        /// \param system offshore system containing this environment
        explicit FrEnvironment_(FrOffshoreSystem_* system);

        /// Destructor
        ~FrEnvironment_();

        /// Get the offshore system, containing this environment
        /// \return offshore system
        FrOffshoreSystem_* GetSystem();

        //---------------------------- Environment scalars methods ----------------------------//

        /// Get the simulation time (given by Chrono)
        /// \return simulation time
        double GetTime() const;

        /// Get the time ramp attached to the environment
        /// \return time ramp
        FrLinearRampFunction_* GetTimeRamp() const;

        /// Get the gravity acceleration on the vertical axis
        /// \return gravity acceleration on the vertical axis, in m/s²
        double GetGravityAcceleration() const;

        /// Set the gravity acceleration on the vertical axis
        /// \param gravityAcceleration gravity acceleration, in m/s²
        void SetGravityAcceleration(double gravityAcceleration);

        /// Return the flow velocity observed from the local frame
        /// \param frame Local frame in which the velocity is computed
        /// \param worldVel Translation velocity of the frame in world frame
        /// \param ft fluid type (AIR/WATER)
        /// \param fc Frame convention (NED/NWU)
        /// \return Velocity in local frame
        Velocity GetRelativeVelocityInFrame(const FrFrame_& frame, const Velocity& worldVel,
                                            FLUID_TYPE ft, FRAME_CONVENTION fc);

        /// Get the fluid density
        /// \param ft fluid type (AIR/WATER)
        /// \return fluid density
        double GetFluidDensity(FLUID_TYPE ft) const;;

        //---------------------------- Environment elements Getters ----------------------------//

        /// Get the Ocean element
        /// \return Ocean element
        FrOcean_* GetOcean() const;

        /// Get the Atmosphere element
        /// \return Atmosphere element
        FrAtmosphere_* GetAtmosphere() const;


        //---------------------------- Geographic coordinates manipulations---------------------------- //

        /// Get the geographic service (convert cartesian to geographic position, compute magnetic declination, etc.)
        /// \return the geographic service
        FrGeographicServices* GetGeographicServices() const;


        //---------------------------- Zoned time conversion manipulations---------------------------- //
        // TODO : ajouter des methodes permettant de recuperer l'heure UTC, de regler le temps origine...

        /// Get the zoned time conversion service
        /// \return zoned time conversion service
        FrTimeZone* GetTimeZone() const;

        /// Get the year given by the zoned time conversion service
        /// \return year
        int GetYear() const;

        //---------------------------- Environment assets hiding helpers ---------------------------- //

        /// (Un)show the free surface
        void ShowFreeSurface(bool show);

        /// (Un)show the seabed
        void ShowSeabed(bool show);


        //---------------------------- Update-Initialize-StepFinalize ---------------------------- //

        /// Update the state of the environment
        void Update(double time);

        /// Initialize the state of the environment
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

    };





}  // end namespace frydom

#endif //FRYDOM_FRENVIRONMENT_H
