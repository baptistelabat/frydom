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


#ifndef FRYDOM_FRTIMEZONE_H
#define FRYDOM_FRTIMEZONE_H

//#include <chrono>
#include "date/tz.h"



namespace frydom {

    /// Class providing services for the conversion of zoned time, based on "date" project : https://github.com/HowardHinnant/date
    /// List of tz database time zones : https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
    class FrTimeZone {

    public:

        enum UTC_LOCAL {
            UTC,
            LOCAL
        };

    private:

        date::zoned_time<std::chrono::milliseconds> m_zonedTime;
        UTC_LOCAL m_UtcOrLocal = LOCAL;
        date::sys_days m_sysDays;
        date::local_days m_localDays;
        std::chrono::milliseconds m_initTime;
        const date::time_zone *m_timeZone = date::current_zone();

    public:

        FrTimeZone();

        ~FrTimeZone() = default;

        void SetSysOrLocal(UTC_LOCAL SL) {m_UtcOrLocal = SL;}

        //auto now() { return std::chrono::system_clock::now(); }

        void SetTimeZone(const std::string &tz_name) {m_timeZone = date::locate_zone(tz_name);}

        void SetDay(int Year, int Month, int Day);

        void SetTime(int Hours, int Minutes, int Seconds);

        void SetZoneDayTime(std::string zoneName, int Year, int Month, int Day, int Hours, int Minutes, int Seconds, UTC_LOCAL SL);

        auto GetSysOrLocal() {return m_UtcOrLocal;}

        double GetTimeZoneOffset(); // returns time zone offset in minutes

        tm* const UTC_to_tm();

        tm* const local_to_tm();

        void Initialize();

        void Update(double time);

        //auto GetZonedTime() { return m_zonedTime; }

        auto GetTimeZone() { return m_timeZone; }

        auto GetLocalTime() { return m_zonedTime.get_local_time(); }

        auto GetUTCTime() { return m_zonedTime.get_sys_time(); }

        date::sys_seconds to_UTC_time(std::tm const &t);

        date::local_seconds to_local_time(std::tm const &t);

    };

}  // end namespace frydom

#endif //FRYDOM_FRTIMEZONE_H
