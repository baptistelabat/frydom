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


#ifndef FRYDOM_FROCEAN_H
#define FRYDOM_FROCEAN_H

#include "frydom/environment/FrFluidType.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrObject.h"
#include "frydom/asset/FrGridAsset.h"

namespace frydom {

    // Forward Declarations
    class FrEnvironment_;
    class FrFreeSurface_;
    class FrCurrent_;
    class FrSeabed_;


    /**
     * \class FrOcean_
     * \brief Class for defining the ocean.
     */
    class FrOcean_ : public FrObject{

    private:

        FrEnvironment_* m_environment;    ///> pointer to the container

        //---------------------------- FrOcean elements ----------------------------//

        std::unique_ptr <FrSeabed_> m_seabed;               ///> Seabed element, with bathymetry model information
        std::unique_ptr <FrFreeSurface_> m_freeSurface;     ///> Free surface element, with tidal, wavefield models information
        std::unique_ptr <FrCurrent_> m_current;             ///> Current, with current model information
        std::unique_ptr <FrFluidProperties> m_waterProp;    ///> Water properties

    public:

        /// Default constructor
        /// \param environment environment containing this ocean
        explicit FrOcean_(FrEnvironment_* environment);

        /// Get the environment containing this ocean
        /// \return environment containing this ocean
        FrEnvironment_* GetEnvironment() const;

        //---------------------------- Assets ----------------------------//

        /// Set if the seabed is to be shown/exist
        /// \param showSeabed showseabed true means the seabed exists
        void ShowSeabed(bool showSeabed);

        /// Set if the free surface is to be shown/exist
        /// \param showFreeSurface showfreesurface true means the free surface exists
        void ShowFreeSurface(bool showFreeSurface);

        //---------------------------- Fluid properties methods ----------------------------//

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
        double GetReynoldsNumberInWater(double characteristicLength, double velocity) const;

        /// Get Froude number (Fe = U/sqrt(g.L) )
        /// \param characteristicLength characteristic length L, in meters
        /// \param velocity fluid velocity U, in m/s
        /// \return Froude number, no dimension
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

        /// Enforce the infinite depth condition on the Seabed object.
        /// A NullSeabed is then considered, with no grid asset and no bathymetry getters
        void SetInfiniteDepth();;

        /// Get mean ocean depth (tidal height + mean bathymetry)
        /// \param fc frame convention (NED/NWU)
        /// \return mean ocean depth, in meters
        double GetDepth(FRAME_CONVENTION fc) const;

        /// Get ocean depth at a position (x,y) (tidal height + bathymetry at position (x,y))
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return ocean depth at position (x,y)
        double GetDepth(double x, double y, FRAME_CONVENTION fc) const;

        //---------------------------- Update-Initialize-StepFinalize ----------------------------//

        /// Update the state of the ocean
        void Update(double time);

        /// Initialize the state of the ocean
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

    };

}  // end namespace frydom

#endif //FRYDOM_FROCEAN_H
