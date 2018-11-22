//
// Created by Lucas Letournel on 22/11/18.
//

#ifndef FRYDOM_FRATMOSPHERE_H
#define FRYDOM_FRATMOSPHERE_H

#include "frydom/environment/FrFluidType.h"
#include "frydom/core/FrVector.h"
#include "frydom/core/FrObject.h"

namespace frydom {

    // Forward Declarations:

    class FrEnvironment_;
    class FrWind_;


    class FrAtmosphere_ : public FrObject{

    private:

        /// pointer to the container
        FrEnvironment_* m_environment;

        /// FrOcean components :
        std::unique_ptr<FrWind_>     m_wind;

        std::unique_ptr <FrFluidProperties> m_airProp;

    public:

        explicit FrAtmosphere_(FrEnvironment_* environment);

        //----------------------------Fluid Properties----------------------------//

        /// Set the fluid temperature
        /// \param Temperature temperature of the fluid
        void SetTemperature(double Temperature);

        /// Get the fluid temperature
        /// \return Temperature temperature of the fluid
        double GetTemperature() const;

        /// Set the fluid density
        /// \param Density density of the fluid
        void SetDensity(double Density);

        /// Get the fluid density
        /// \return Density density of the fluid
        double GetDensity() const;

        /// Set the fluid dynamic viscosity
        /// \param DynamicViscosity dynamic viscosity of the fluid
        void SetDynamicViscosity(double DynamicViscosity);

        /// Get the fluid dynamic viscosity
        /// \return DynamicViscosity dynamic viscosity of the fluid
        double GetDynamicViscosity() const;

        /// Set the fluid kinematic viscosity
        /// \param KinematicViscosity kinematic viscosity of the fluid
        void SetKinematicViscosity(double KinematicViscosity);

        /// Get the fluid kinematic viscosity
        /// \return KinematicViscosity kinematic viscosity of the fluid
        double GetKinematicViscosity() const;

        /// Set the fluid salinity
        /// \param Salinity salinity of the fluid
        void SetSalinity(double Salinity);

        /// Get the fluid salinity
        /// \return Salinity salinity of the fluid
        double GetSalinity() const;

        /// Set the fluid pressure
        /// \param Pressure pressure of the fluid
        void SetPressure(double Pressure);

        /// Get the fluid pressure
        /// \return Pressure pressure of the fluid
        double GetPressure() const;

        double GetReynoldsNumberInAir(double characteristicLength, double velocity) const;

        double GetFroudeNumberInAir(double characteristicLength, double velocity) const;

        // Ocean elements Getters

        /// Get The wind element
        /// \return the wind element
        FrWind_* GetWind() const;

        void Update(double time);

        void Initialize() override;

        void StepFinalize() override;

    };
}

#endif //FRYDOM_FRATMOSPHERE_H
