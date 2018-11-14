//
// Created by frongere on 10/07/17.
//

#ifndef FRYDOM_FRENVIRONMENT_H
#define FRYDOM_FRENVIRONMENT_H


#include "frydom/core/FrObject.h"

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
#include "frydom/utils/FrGeographicServices.h"
#include "frydom/core/FrGeographic.h"
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
            m_geoServices->Convert_GeoToCart(lat,lon,h,x,y,z);
        }

        void Convert_CartToGeo(double x, double y, double z, double& lat, double& lon, double& h){
            m_geoServices->Convert_CartToGeo(x,y,z,lat,lon,h);
        }

        double ComputeMagneticDeclination(double x, double y, double z){
            m_geoServices->ComputeMagneticDeclination(x,y,z,GetYear());
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














    /// REFACTORING ---->>>>







    class FrOffshoreSystem_;
    class FrFreeSurface_;
    class FrTidal_;
    class FrSeabed_;
    class FrUniformWind_;
    class FrUniformCurrent_;
    class FrFlowBase;
    class FrWind_;
    class FrCurrent_;
    class FrCurrent_;
    class FrWind_;
    class Velocity;
    class FrFrame_;


    /// Class to store the different elements composing the offshore environment
    class FrEnvironment_ : public FrObject {

    private:

        FrOffshoreSystem_* m_system;

//        double m_time;
        std::unique_ptr<FrTimeZone> m_timeZone;  // TODO : faire un service de temps

        // Environment components
        std::unique_ptr<FrFreeSurface_> m_freeSurface;
        std::unique_ptr<FrFlowBase>     m_current;  // TODO: remettre en place de la genericite
        std::unique_ptr<FrFlowBase>     m_wind;
        std::unique_ptr<FrSeabed_>      m_seabed;

        /// Structure for converting local coordinates to geographic coordinates, contains the geocoord origins
        std::unique_ptr<GeographicLib::LocalCartesian> m_LocalCartesian;

        // Environments scalars
        double m_waterDensity = 1025.;
        double m_airDensity = 1.204;

//        double m_gravityAcceleration = 9.81;

        double m_seaTemperature = 15.;
        double m_airTemperature = 20.;

        double m_waterKinematicViscosity;

        double m_atmosphericPressure;

//        bool m_infinite_depth = false; // TODO : a placer plutot dans seabed !

        bool m_showSeabed = true;
        bool m_showFreeSurface = true;

    public:

        explicit FrEnvironment_(FrOffshoreSystem_* system);

        ~FrEnvironment_();

//        void SetSystem(FrOffshoreSystem* system) {
//            m_system = system;
//        }

        FrOffshoreSystem_* GetSystem();
//
//        void SetInfiniteDepth(bool is_infinite);
//        bool GetInfiniteDepth();

        double GetTime() const;


        // Environment scalars

        double GetWaterDensity() const;

        void SetWaterDensity(double waterDensity);

        double GetAirDensity() const;

        void SetAirDensity(double airDensity);

        double GetFluidDensity(FLUID_TYPE ft) const;

        double GetGravityAcceleration() const;

        void SetGravityAcceleration(double gravityAcceleration);

        double GetSeaTemperature() const;

        void SetSeaTemperature(double seaTemperature);

        double GetAirTemperature() const;

        void SetAirTemperature(double airtemperature);

        double GetWaterKinematicViscosity() const;

        void SetWaterKinematicViscosity(double waterKinematicViscosity);

        double GetAtmosphericPressure() const;

        void SetAtmosphericPressure(double atmosphericPressure);

        double GetReynoldsNumberInWater(double characteristicLength, double velocity) const;

        double GetFroudeNumberInWater(double characteristicLength, double velocity) const;


        // Environment elements Getters

        FrFreeSurface_* GetFreeSurface() const;

        FrTidal_* GetTidal() const;

        FrCurrent_* GetCurrent() const;

        FrWind_* GetWind() const;

//        void SetCurrent(FrCurrent* current);
//
//        void SetCurrent (const FrCurrent::MODEL type=FrCurrent::UNIFORM);

//        void SetWind(const FrWind::MODEL type=FrWind::UNIFORM);

        FrSeabed_* GetSeabed() const;

        Velocity GetRelativeVelocityInFrame(const FrFrame_& frame, const Velocity& worldVel,
                FLUID_TYPE ft, FRAME_CONVENTION fc);

//        void SetSeabed(FrSeabed* seabed);


        // Geographic coordinates manipulations

        GeographicLib::LocalCartesian* GetGeoLib() const;

        void SetGeographicOrigin(double lat0, double lon0, double h0);

        void Convert_GeoToCart(double lat, double lon, double h, double& x, double& y, double& z);

        void Convert_CartToGeo(double x, double y, double z, double& lat, double& lon, double& h);

        int GetYear() const;        

        // Earth magnetic model

        double ComputeMagneticDeclination(double x, double y, double z); // Local position (cartesian)


        // TODO : ajouter des methodes permettant de recuperer l'heure UTC, de regler le temps origine...

        FrTimeZone* GetTimeZone() const;
        //void SetTimeZoneName(FrTimeZone* TimeZone) {m_timeZoneName = TimeZone;}


        // Solver methods

        void Update(double time);

        void Initialize() override;

        void StepFinalize() override;

    };







}  // end namespace frydom

#endif //FRYDOM_FRENVIRONMENT_H
