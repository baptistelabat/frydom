//
// Created by Lucas Letournel on 15/06/18.
//

#ifndef FRYDOM_FRTIMEZONE_H
#define FRYDOM_FRTIMEZONE_H

#include <chrono>
#include "include/date/tz.h"

/// List of tz database time zones : https://en.wikipedia.org/wiki/List_of_tz_database_time_zones

namespace frydom {

    class FrTimeZone {
    public:
        enum UtcOrLocal {utc, local};
    private:
        date::zoned_time<std::chrono::milliseconds> m_zonedTime;
        UtcOrLocal m_UtcOrLocal = local;
        date::sys_days m_sysDays;
        date::local_days m_localDays;
        std::chrono::milliseconds m_initTime;
        const date::time_zone *m_timeZone;
    public:

        void SetSysOrLocal(UtcOrLocal SL) {m_UtcOrLocal = SL;}

        //auto now() { return std::chrono::system_clock::now(); }

        void SetTimeZone(const std::string &tz_name) {m_timeZone = date::locate_zone(tz_name);}

        void SetDay(int Year, int Month, int Day);

        void SetTime(int Hours, int Minutes, int Seconds);

        void SetZoneDayTime(std::string zoneName, int Year, int Month, int Day, int Hours, int Minutes, int Seconds, UtcOrLocal SL);

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

}
#endif //FRYDOM_FRTIMEZONE_H
