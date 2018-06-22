//
// Created by Lucas Letournel on 15/06/18.
//

#ifndef FRYDOM_FRTIMEZONE_H
#define FRYDOM_FRTIMEZONE_H

#include <chrono>
#include "include/date/tz.h"

/// List of tz database time zones : https://en.wikipedia.org/wiki/List_of_tz_database_time_zones

using namespace std::chrono;
using namespace date;

namespace frydom {

    class FrTimeZone {
    public:
        enum SysOrLocal {sys, local};
    private:
        zoned_time<milliseconds> m_zonedTime;
        SysOrLocal m_sysOrLocal = local;
        date::sys_days m_sysDays;
        date::local_days m_localDays;
        std::chrono::milliseconds m_initTime;
        std::string m_timeZoneName;
    public:

        void SetSysOrLocal(SysOrLocal SL) {m_sysOrLocal = SL;}


        auto now() { return std::chrono::system_clock::now(); }

        void SetTimeZoneName(const std::string &tz_name) {m_timeZoneName = tz_name;}

        void SetZonedTime(zoned_time<duration<long>> zonedtime) {m_zonedTime = zonedtime;}

        void SetDay(int Year, int Month, int Day);

        void SetTime(int Hours, int Minutes, int Seconds);

        void SetZoneDayTime(std::string zoneName, int Year, int Month, int Day, int Hours, int Minutes, int Seconds, SysOrLocal SL);

        auto GetZonedTime() { return m_zonedTime; }

        auto GetTimeZone() { return m_zonedTime.get_time_zone(); }

        auto GetLocalTime() { return m_zonedTime.get_local_time(); }

        auto GetSysTime() { return m_zonedTime.get_sys_time(); }

        auto GetSysOrLocal() {return m_sysOrLocal;}

        std::chrono::seconds GetTimeZoneOffset();

        tm to_tm();//date::zoned_time<milliseconds> tp

        date::sys_seconds to_sys_time(std::tm const &t);

        date::local_seconds to_local_time(std::tm const &t);

        void Initialize();

        void Update(double time);

    };

}
#endif //FRYDOM_FRTIMEZONE_H
