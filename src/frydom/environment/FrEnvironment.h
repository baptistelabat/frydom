//
// Created by frongere on 10/07/17.
//

#ifndef FRYDOM_FRENVIRONMENT_H
#define FRYDOM_FRENVIRONMENT_H

//#include <frydom/environment/wind/FrWind.h>
#include "frydom/core/FrObject.h"

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
//#include <GeographicLib/LocalCartesian.hpp>


//#include "frydom/core/FrOffshoreSystem.h"
// TODO: passer le max en forward declaration !!

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
    class FrUniformCurrent;
    class FrWind;
    class FrUniformWind;

    class FrSeabed;




    /// Class to store the different elements composing the offshore environment
    class FrEnvironment : public FrObject {

    private:

        FrOffshoreSystem* m_system;

        double m_time;
        std::unique_ptr<FrTimeZone> m_timeZone;

        // Environment components
        std::unique_ptr<FrFreeSurface> m_freeSurface;
        std::shared_ptr<FrCurrent> m_current;
        std::shared_ptr<FrWind> m_wind;
        std::unique_ptr<FrSeabed> m_seabed;

        /// Structure for converting local coordinates to geographic coordinates, contains the geocoord origins
        std::unique_ptr<GeographicLib::LocalCartesian> m_LocalCartesian;

        // Environments scalars
        double m_waterDensity = 1025.;
        double m_airDensity = 1.204;

        double m_gravityAcceleration = 9.81;

        double m_seaTemperature = 15.;
        double m_airtemperature = 20.;

        double m_waterKinematicViscosity;

        double m_atmosphericPressure;

        bool m_infinite_depth = false;


    public:

        FrEnvironment();

        void SetSystem(FrOffshoreSystem* system);

        FrOffshoreSystem* GetSystem();

        void SetInfiniteDepth(bool is_infinite);
        bool GetInfiniteDepth();

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

        void SetFreeSurface(FrFreeSurface* freeSurface);

        FrTidal* GetTidal() const;
//
//        void SetTidal(FrTidal* tidal) {
//            m_tidal = std::unique_ptr<FrTidal>(tidal);
//        }

        template <class T=FrUniformCurrent>
        T* GetCurrent() const;

        template <class T=FrUniformWind>
        T* GetWind() const;

        void SetCurrent(FrCurrent* current);

//        void SetCurrent (const FrCurrent::MODEL type=FrCurrent::UNIFORM);
//
//        void SetWind(const FrWind::MODEL type=FrWind::UNIFORM);

        inline FrSeabed* GetSeabed() const;

        void SetSeabed(FrSeabed* seabed);

        GeographicLib::LocalCartesian* GetGeoLib() const;

        void SetGeographicOrigin(double lat0, double lon0, double h0);

        void Convert_GeoToCart(double lat, double lon, double h, double& x, double& y, double& z);

        void Convert_CartToGeo(double x, double y, double z, double& lat, double& lon, double& h);

        double ComputeMagneticDeclination(double x, double y, double z);

        FrTimeZone* GetTimeZone() const;
        //void SetTimeZoneName(FrTimeZone* TimeZone) {m_timeZoneName = TimeZone;}



        void Update(double time);

        void Initialize() override;

        void StepFinalize() override;

    };









    /// REFACTORING ---->>>>

    class FrOffshoreSystem_;
    class FrFreeSurface_;
    class FrTidal_;
    class FrSeabed_;
    class FrCurrent_;
    class FrWind_;


    /// Class to store the different elements composing the offshore environment
    class FrEnvironment_ : public FrObject {

    private:

        FrOffshoreSystem_* m_system;

//        double m_time;
        std::unique_ptr<FrTimeZone> m_timeZone;

        // Environment components
        std::unique_ptr<FrFreeSurface_>  m_freeSurface;
        std::shared_ptr<FrCurrent_>       m_current;
        std::shared_ptr<FrWind_>          m_wind;
        std::unique_ptr<FrSeabed_>       m_seabed;

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


    public:

        explicit FrEnvironment_(FrOffshoreSystem_* system);

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


        // Environment elements Getters

        FrFreeSurface_* GetFreeSurface() const;

        FrTidal_* GetTidal() const;

        template <class T=FrUniformCurrent>
        T* GetCurrent() const;

        template <class T=FrUniformWind>
        T* GetWind() const;

//        void SetCurrent(FrCurrent* current);
//
//        void SetCurrent (const FrCurrent::MODEL type=FrCurrent::UNIFORM);

//        void SetWind(const FrWind::MODEL type=FrWind::UNIFORM);

        FrSeabed_* GetSeabed() const;

//        void SetSeabed(FrSeabed* seabed);


        // Geographic coordinates manipulations

        GeographicLib::LocalCartesian* GetGeoLib() const;

        void SetGeographicOrigin(double lat0, double lon0, double h0);

        void Convert_GeoToCart(double lat, double lon, double h, double& x, double& y, double& z);

        void Convert_CartToGeo(double x, double y, double z, double& lat, double& lon, double& h);


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
