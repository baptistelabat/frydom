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
    private:
        time_zone *m_timeZone;
        zoned_time<duration<long>> m_zonedTime;
    public:
        auto now() { return std::chrono::system_clock::now(); }

        //auto current_zone() { return date::current_zone(); }

        auto LocateZone(const std::string &tz_name) { return date::locate_zone(tz_name); }

        auto LocalTime(const time_zone* zone, const sys_time<duration<long>>& st) {return date::make_zoned(zone, st);}

        void SetTimeZone(time_zone* timezone) {m_timeZone = timezone;}

        void SetZonedTime(zoned_time<duration<long>> zonedtime) {m_zonedTime = zonedtime;}

        std::string GetName() { return m_timeZone->name(); }

        auto GetTimeZone() { return m_timeZone; }

        auto GetZonedTime() { return m_zonedTime; }

        auto GetLocalTime() { return m_zonedTime.get_local_time(); }

        auto GetSysTime() { return m_zonedTime.get_sys_time(); }

        std::tm to_tm(date::zoned_seconds tp);

        date::sys_seconds to_sys_time(std::tm const &t);

        date::local_seconds to_local_time(std::tm const &t);


    };

}
#endif //FRYDOM_FRTIMEZONE_H
