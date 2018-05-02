//
// Created by frongere on 10/07/17.
//

#ifndef FRYDOM_FRENVIRONMENT_H
#define FRYDOM_FRENVIRONMENT_H

#include "frydom/core/FrObject.h"

// Current includes

#include "current/FrCurrent.h"
#include "current/FrCurrentPolarCoeffs.h"
#include "current/FrCurrentForce.h"

// Waves includes
#include "waves/FrFreeSurface.h"
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
    class FrEnvironment : public FrObject {

    private:

        FrOffshoreSystem* m_system;

        double m_time;

        // Environment components
        std::unique_ptr<FrFreeSurface> m_freeSurface;
        std::shared_ptr<FrCurrent> m_current;
        std::shared_ptr<FrWind> m_wind;
        std::unique_ptr<FrSeabed> m_seabed;

        // Environments scalars
        double m_waterDensity = 1025.;
        double m_airDensity = 1.204;

        double m_gravityAcceleration = 9.81;

        double m_seaTemperature = 15.;
        double m_airtemperature = 20.;

        double m_waterKinematicViscosity;

        double m_atmosphericPressure;


    public:

        FrEnvironment() {

            m_freeSurface = std::make_unique<FrFreeSurface>();
            m_current = std::make_shared<FrUniformCurrent>();
            m_wind = std::make_shared<FrUniformWind>();
            m_seabed = std::make_unique<FrSeabed>();

        }

        void SetSystem(FrOffshoreSystem* system) {
            m_system = system;
        }

        FrOffshoreSystem* GetSystem() { return m_system; }

        void SetTime(double time) { m_time = time; }

        double GetTime() const { return m_time; }

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
            // TODO: gerer la temperature
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

        inline FrFreeSurface* GetFreeSurface() const {
            return m_freeSurface.get();
        }

        void SetFreeSurface(FrFreeSurface* freeSurface);

        FrTidal* GetTidal() const {
            return m_freeSurface->GetTidal();
        }
//
//        void SetTidal(FrTidal* tidal) {
//            m_tidal = std::unique_ptr<FrTidal>(tidal);
//        }

        template <class T=FrUniformCurrent>
        T* GetCurrent() const { return dynamic_cast<T*>(m_current.get()); }

        template <class T=FrUniformWind>
        T* GetWind() const { return dynamic_cast<T*>(m_wind.get()); }

        void SetCurrent(FrCurrent* current) {
            m_current = std::shared_ptr<FrCurrent>(current);
        }

        void SetCurrent (const FrCurrent::MODEL type=FrCurrent::UNIFORM) {

            switch (type) {
                case FrCurrent::UNIFORM:
                        m_current = std::make_shared<FrUniformCurrent>();
                        break;
                default:
                        break;
                }
        }

        void SetWind(const FrWind::MODEL type=FrWind::UNIFORM) {

            switch (type) {
                case FrWind::UNIFORM:
                    m_wind = std::make_shared<FrUniformWind>();
                    break;
                default:
                    break;
            }
        }

        inline FrSeabed* GetSeabed() const {
            return m_seabed.get();
        }

        void SetSeabed(FrSeabed* seabed) {
            m_seabed = std::unique_ptr<FrSeabed>(seabed);
        }

        void Update(double time) {
            m_freeSurface->Update(time);
            m_current->Update(time);
            m_wind->Update(time);
            m_seabed->Update(time);
            m_time = time;
        }

        void Initialize() override {
            // TODO: appeler les methodes Initialize() sur les attributs
            m_freeSurface->Initialize();
            m_current->Initialize();
            m_wind->Initialize();
            m_seabed->Initialize();
        }

        void StepFinalize() override {
            m_freeSurface->StepFinalize();
            m_current->StepFinalize();
            m_wind->StepFinalize();
            m_seabed->StepFinalize();
        }

    };

}  // end namespace frydom

#endif //FRYDOM_FRENVIRONMENT_H
