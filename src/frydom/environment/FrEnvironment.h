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

#include <memory>

#include "frydom/core/common/FrObject.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/environment/FrFluidType.h"


// Forward declaration
namespace GeographicLib {
    class LocalCartesian;
}

namespace frydom {

    // Forward declarations
    class FrTimeServices;
    class FrOffshoreSystem;
    class FrOcean;
    class FrAtmosphere;
    class FrGeographicServices;
    class Velocity;
    class FrFrame;
    class FrCosRampFunction;


    /**
     * \class FrEnvironment
     * \brief Class for defining the environmental data.
     */
    class FrEnvironment : public FrObject {

    private:

        FrOffshoreSystem* m_system;    ///< Offshore sytem containing this Environment

        //---------------------------- Environment elements ----------------------------//
        // TODO : faire un service de temps, NEED REFACTO
        std::unique_ptr<FrTimeServices> m_timeServices;                 ///< Zoned time conversion service, can give time during simulation in a specified time zone.

        std::unique_ptr<FrCosRampFunction> m_timeRamp;                  ///< Time ramp, can be applied on wave field, current field, wind field, etc.

        std::unique_ptr<FrOcean> m_ocean;                              ///> Ocean element of the simulation, contains free surface and seabed, current model, water properties, etc.

        std::unique_ptr<FrAtmosphere> m_atmosphere;                    ///> Atmosphere element of the simulation, contains wind model, air properties.

        std::unique_ptr<FrGeographicServices> m_geographicServices;    ///> Service converting local coordinates to geographic coordinates, contains the geocoord origins.

    public:

        /// Default constructor
        /// \param system offshore system containing this environment
        explicit FrEnvironment(FrOffshoreSystem* system);

        /// Destructor
        ~FrEnvironment();

        /// Get the offshore system, containing this environment
        /// \return offshore system
        FrOffshoreSystem* GetSystem();

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "Environment"; }

        //---------------------------- Environment scalars methods ----------------------------//

        /// Get the simulation time (given by Chrono)
        /// \return simulation time
        double GetTime() const;

        /// Get the time ramp attached to the environment
        /// \return time ramp
        FrCosRampFunction* GetTimeRamp() const;

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
        Velocity GetRelativeVelocityInFrame(const FrFrame& frame, const Velocity& worldVel,
                                            FLUID_TYPE ft, FRAME_CONVENTION fc);

        /// Get the fluid density
        /// \param ft fluid type (AIR/WATER)
        /// \return fluid density
        double GetFluidDensity(FLUID_TYPE ft) const;;

        //---------------------------- Environment elements Getters ----------------------------//

        /// Get the Ocean element
        /// \return Ocean element
        FrOcean* GetOcean() const;

        /// Get the Atmosphere element
        /// \return Atmosphere element
        FrAtmosphere* GetAtmosphere() const;


        //---------------------------- Geographic coordinates manipulations---------------------------- //

        /// Get the geographic service (convert cartesian to geographic position, compute magnetic declination, etc.)
        /// \return the geographic service
        FrGeographicServices* GetGeographicServices() const;


        //---------------------------- Zoned time conversion manipulations---------------------------- //
        // TODO : ajouter des methodes permettant de recuperer l'heure UTC, de regler le temps origine...

        /// Get the zoned time conversion service
        /// \return zoned time conversion service
        FrTimeServices* GetTimeServices() const;

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
