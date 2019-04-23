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


#ifndef FRYDOM_FRTIMESERVICES_H
#define FRYDOM_FRTIMESERVICES_H

#include <date/tz.h>

namespace frydom {

    // Forward declarations

    /**
    * \class FrTimeServices
    * \brief Class providing services related to zoned time.
    * It includes the library date.tz by Howard Hinnant : https://github.com/HowardHinnant/date
    * TThe list of tz database time zones can be found at https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
    */
    class FrTimeServices {

    private:
        std::chrono::milliseconds c_time;       ///< cached value of the time (m_initTime + simulationTime)

        std::chrono::milliseconds m_initTime;   ///< starting time of the simulation, can be user defined

        date::local_days m_initDay;             ///< starting day of the simulation, can be user defined

    public:

        /// Constructor of the TimeServices provider
        FrTimeServices();

        /// Default destructor
        ~FrTimeServices() = default;

//        void SetStartingDay(int Year, int Month, int Day);

        /// Set the starting day of the simulation
        /// \param Day starting day of the simulation
        void SetStartingDay(date::local_days Day);

        /// Set the starting day of the simulation, using : year, month, day convention
        /// \param Year starting year of the simulation
        /// \param Month starting month of the simulation
        /// \param Day starting day of the simulation
        void SetStartingDay(date::year Year, date::month Month, date::day Day);

//        void SetStartingTime(int Hours, int Minutes, int Seconds);

        /// Set the starting time of the simulation since midnight, using : hours, minutes, seconds convention
        /// \param Hours starting hour of the simulation
        /// \param Minutes starting minute of the simulation
        /// \param Seconds starting second of the simulation
        void SetStartingTime(std::chrono::hours Hours, std::chrono::minutes Minutes, std::chrono::seconds Seconds);

        /// Set the starting time of the simulation since midnight, in milliseconds
        /// \param time starting time of the simulation since midnight, in milliseconds
        void SetStartingTime(std::chrono::milliseconds time);

        /// Set the starting time of the simulation (not since midnight !), as a zoned time duration
        /// \param time starting time of the simulation
        void SetStartingTime(date::zoned_time<std::chrono::milliseconds> time);

        /// Get the current simulation time, in the specified time zone
        /// \param timeZone time zone, in which returning the simulation time
        /// \return Current simulation time
        auto GetTime(const date::time_zone* timeZone = date::current_zone()) const {
            return date::make_zoned(timeZone, m_initDay + c_time).get_local_time();
        }

        /// Get the current simulation time, in the specified time zone
        /// \param timeZoneName name of the time zone, see https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
        /// \return Current simulation time
        auto GetTime(const std::string &timeZoneName) const {
            //TODO: générer erreur si mauvais nom de timezone
            auto timeZone = date::locate_zone(timeZoneName);
            return GetTime(timeZone);
        }

        /// Get the UTC simulation time
        /// \return UTC simulation time
        auto GetUTCTime() const {
            auto UTC = date::locate_zone("UTC");
            return GetTime(UTC);
        }

        /// Get the time zone offset, in minutes, of the specified time zone, compared to UTC
        /// \param timeZoneName name of the time zone, see https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
        /// \return time zone offset, in minutes
        double GetTimeZoneOffset(const std::string &timeZoneName); // returns time zone offset in minutes


        /// Get the time zone offset, in minutes, of the specified time zone, compared to UTC
        /// \param timeZone time zone, see https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
        /// \return time zone offset, in minutes
        double GetTimeZoneOffset(const date::time_zone* timeZone); // returns time zone offset in minutes

        /// Initialize the cached value of the simulation time to the starting time
        void Initialize();

        /// Update the cached valued of the simulation time
        void Update(double time);

    };

}
#endif //FRYDOM_FRTIMESERVICES_H
