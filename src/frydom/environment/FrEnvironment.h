//
// Created by frongere on 10/07/17.
//

#ifndef FRYDOM_FRENVIRONMENT_H
#define FRYDOM_FRENVIRONMENT_H


// Current includes

#include "current/FrCurrent.h"
#include "current/FrCurrentPolarCoeffs.h"
#include "current/FrCurrentForce.h"

// Waves includes
#include "waves/FrFlatFreeSurface.h"
#include "waves/FrWaveSpectrum.h"
#include "waves/FrWaveField.h"
#include "tidal/FrTidalModel.h"

// Wind includes
#include "wind/FrWind.h"
#include "wind/FrWindForce.h"

// Seabed includes
#include "seabed/FrSeabed.h"

namespace frydom {
    /// Class to store the different elements composing the offshore environment
    class FrEnvironment {

    private:

        FrOffshoreSystem* m_system;

        std::unique_ptr<FrFreeSurface> m_freeSurface;
        std::unique_ptr<FrTidal> m_tidal;
        std::unique_ptr<FrCurrent> m_current;
        std::unique_ptr<FrWind> m_wind;
        std::unique_ptr<FrSeabed> m_seabed;

        double m_waterDensity = 1025.;
        double m_airDensity = 1.204;

        double m_gravityAcceleration = 9.81;

        double m_seaTemperature = 15.;
        double m_airtemperature = 20.;

        double m_waterKinematicViscosity;

        double m_atmosphericPressure;


    public:
        FrEnvironment() {

            m_freeSurface = std::make_unique<FrFlatFreeSurface>(0.);
            m_tidal = std::make_unique<FrTidal>();
            m_current = std::make_unique<FrCurrent>();
            m_wind = std::make_unique<FrWind>();
            m_seabed = std::make_unique<FrSeabed>();

        }

        void SetSystem(FrOffshoreSystem* system) {
            m_system = system;
        }

        FrOffshoreSystem* GetSystem() { return m_system; }

        double GetWaterDensity() const {
            return m_waterDensity;
        }

        void SetWaterDensity(const double waterDensity) {
            m_waterDensity = waterDensity;
        }

        double GetAirDensity() const {
            return m_airDensity;
        }

        void SetAirDensity(double airDensity) {
            m_airDensity = m_airDensity;
        }

        double GetGravityAcceleration() const {
            return m_gravityAcceleration;
        }

        void SetGravityAcceleration(double gravityAcceleration) {
            m_gravityAcceleration = m_gravityAcceleration;
        }

        double GetSeaTemperature() const {
            return m_seaTemperature;
        }

        void SetSeaTemperature(double seaTemperature) {
            m_seaTemperature = m_seaTemperature;
        }

        double GetAirtemperature() const {
            return m_airtemperature;
        }

        void SetAirtemperature(double airtemperature) {
            m_airtemperature = m_airtemperature;
        }

        double GetWaterKinematicViscosity() const {
            return m_waterKinematicViscosity;
        }

        void SetWaterKinematicViscosity(double waterKinematicViscosity) {
            m_waterKinematicViscosity = m_waterKinematicViscosity;
        }

        double GetAtmosphericPressure() const {
            return m_atmosphericPressure;
        }

        void SetAtmosphericPressure(double atmosphericPressure) {
            m_atmosphericPressure = m_atmosphericPressure;
        }

        FrFreeSurface* GetFreeSurface() const {
            return m_freeSurface.get();
        }

        void SetFreeSurface(FrFreeSurface* freeSurface);

        FrTidal* GetTidal() const {
            return m_tidal.get();
        }

        void SetTidal(FrTidal* tidal) {
            m_tidal = std::unique_ptr<FrTidal>(tidal);
        }

        FrCurrent* GetCurrent() const {
            return m_current.get();
        }

        void SetCurrent(FrCurrent* current) {
            m_current = std::unique_ptr<FrCurrent>(current);
        }

        FrWind* GetWind() const {
            return m_wind.get();
        }

        void SetWind(FrWind* wind) {
            m_wind = std::unique_ptr<FrWind>(wind);
        }

        FrSeabed* GetSeabed() const {
            return m_seabed.get();
        }

        void SetSeabed(FrSeabed* seabed) {
            m_seabed = std::unique_ptr<FrSeabed>(seabed);
        }

    };

}  // end namespace frydom

#endif //FRYDOM_FRENVIRONMENT_H
