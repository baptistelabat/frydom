// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRATMOSPHERE_H
#define FRYDOM_FRATMOSPHERE_H

#include "frydom/environment/FrFluidType.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrObject.h"

namespace frydom {

    // Forward Declarations:

    class FrEnvironment_;
    class FrWind_;


    /**
     * \class FrAtmosphere_
     * \brief Class for defining the atmosphere.
     */
    class FrAtmosphere_ : public FrObject{

    private:

        FrEnvironment_* m_environment;    ///< pointer to the container

        //---------------------------- Atmosphere elements ----------------------------//

        std::unique_ptr<FrWind_>     m_wind;    ///< Wind, with wind model information
        std::unique_ptr <FrFluidProperties> m_airProp;  ///< Air properties

    public:

        /// Default constructor
        /// \param environment environment containing this atmosphere
        explicit FrAtmosphere_(FrEnvironment_* environment);

        /// Get the environment containing this atmosphere
        /// \return environment containing this atmosphere
        FrEnvironment_* GetEnvironment() const;

        //----------------------------Fluid properties methods----------------------------//

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

        /// Get Reynolds number (Re = U.L/nu)
        /// \param characteristicLength characteristic length L, in meters
        /// \param velocity fluid velocity U, in m/s
        /// \return Reynolds number, no dimension
        double GetReynoldsNumberInAir(double characteristicLength, double velocity) const;

        /// Get Froude number (Fe = U/sqrt(g.L) )
        /// \param characteristicLength characteristic length L, in meters
        /// \param velocity fluid velocity U, in m/s
        /// \return Froude number, no dimension
        double GetFroudeNumberInAir(double characteristicLength, double velocity) const;

        //---------------------------- Ocean elements Getters ----------------------------//

        /// Get The wind element
        /// \return the wind element
        FrWind_* GetWind() const;

        //---------------------------- Update-Initialize-StepFinalize ----------------------------//

        /// Update the state of the atmosphere
        void Update(double time);

        /// Initialize the state of the atmosphere
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

    };
}

#endif //FRYDOM_FRATMOSPHERE_H
