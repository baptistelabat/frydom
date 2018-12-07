//
// Created by Lucas Letournel on 22/11/18.
//

#ifndef FRYDOM_FROCEAN_H
#define FRYDOM_FROCEAN_H

#include "frydom/environment/FrFluidType.h"
#include "frydom/core/FrVector.h"
#include "frydom/core/FrObject.h"
#include "frydom/asset/FrGridAsset.h"

namespace frydom {

    // Forward Declarations:

    class FrEnvironment_;
    class FrFreeSurface_;
    class FrCurrent_;
    class FrSeabed_;


    class FrOcean_ : public FrObject{

    private:

        /// pointer to the container
        FrEnvironment_* m_environment;

        /// FrOcean components :
        std::unique_ptr <FrSeabed_> m_seabed;
        std::unique_ptr <FrFreeSurface_> m_freeSurface;
        std::unique_ptr <FrCurrent_> m_current;

        std::unique_ptr <FrFluidProperties> m_waterProp;

    public:

        explicit FrOcean_(FrEnvironment_* environment);

        FrEnvironment_* GetEnvironment() const;

        double GetTime() const;

        //---------------------------- Assets ----------------------------//
        void ShowSeabed(bool showSeabed);
        void ShowFreeSurface(bool showFreeSurface);

        //---------------------------- Fluid Properties ----------------------------//

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

        double GetDepth() const;

        double GetDepth(double x, double y) const;

        double GetReynoldsNumberInWater(double characteristicLength, double velocity) const;

        double GetFroudeNumberInWater(double characteristicLength, double velocity) const;

        //---------------------------- Ocean elements Getters ----------------------------//

        /// Get the free surface element
        /// \return the free surface element
        FrFreeSurface_* GetFreeSurface() const;

        /// Get The current element
        /// \return the current element
        FrCurrent_* GetCurrent() const;

        /// Get the seabed element
        /// \return the seabed element
        FrSeabed_* GetSeabed() const;

        //---------------------------- Update-Initialize-StepFinalize ----------------------------//

        void Update(double time);

        void Initialize() override;

        void StepFinalize() override;

    };
}

#endif //FRYDOM_FROCEAN_H
