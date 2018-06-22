//
// Created by Lucas Letournel on 15/06/18.
//

#include "FrTimeZone.h"
#include <date/tz.h>
#include "fmt/format.h"

namespace frydom {
    tm FrTimeZone::to_tm() {//date::zoned_time<milliseconds> tp
        /// FROM https://github.com/HowardHinnant/date/wiki/Examples-and-Recipes#converting-to-a-tm
        auto lt = m_zonedTime.get_local_time();
        auto ld = floor<days>(lt);
        time_of_day<milliseconds> tod{lt - ld};  // <seconds> can be omitted in C++17
        year_month_day ymd{ld};
        tm t{};
        t.tm_sec = tod.seconds().count();
        t.tm_min = tod.minutes().count();
        t.tm_hour = tod.hours().count();
        t.tm_mday = (ymd.day() - 0_d).count();
        t.tm_mon = (ymd.month() - January).count();
        t.tm_year = (ymd.year() - 1900_y).count();
        t.tm_wday = (weekday{ld} - Sunday).count();
        t.tm_yday = (ld - local_days{ymd.year() / jan / 1}).count();
        t.tm_isdst = m_zonedTime.get_info().save != minutes{0};
        return t;
    }

    date::sys_seconds FrTimeZone::to_sys_time(std::tm const &t) {
        return date::sys_days{date::year(t.tm_year+1900)/date::month(t.tm_mon+1)/date::day(t.tm_mday)}
               + std::chrono::hours{t.tm_hour} + std::chrono::minutes{t.tm_min} + std::chrono::seconds{t.tm_sec};
    }

    date::local_seconds FrTimeZone::to_local_time(std::tm const &t) {
        return date::local_days{date::year(t.tm_year+1900)/date::month(t.tm_mon+1)/date::day(t.tm_mday)}
               + std::chrono::hours{t.tm_hour} + std::chrono::minutes{t.tm_min} + std::chrono::seconds{t.tm_sec};
    }

    void FrTimeZone::SetZoneDayTime(std::string zoneName, int Year, int Month, int Day, int Hours, int Minutes,
                                    int Seconds, SysOrLocal SL) {
        SetSysOrLocal(SL);
        SetTimeZoneName(zoneName);
        SetDay(Year, Month, Day);
        SetTime(Hours,Minutes,Seconds);
    }

    void FrTimeZone::SetDay(int Year, int Month, int Day) {
        switch (m_sysOrLocal) {
            case sys :
                m_sysDays = date::sys_days{date::year(Year)/date::month(Month)/date::day(Day)};
                break;
            case local :
                m_localDays = date::local_days{date::year(Year)/date::month(Month)/date::day(Day)};
                break;
        }
    }

    void FrTimeZone::SetTime(int Hours, int Minutes, int Seconds) {
        m_initTime = std::chrono::hours(Hours) + std::chrono::minutes(Minutes) + std::chrono::seconds(Seconds);
    }

    std::chrono::seconds FrTimeZone::GetTimeZoneOffset() {
        return GetTimeZone()->get_info(GetSysTime()).offset;
    }

    void FrTimeZone::Initialize() {
        auto Zone = date::locate_zone(m_timeZoneName);
        switch (m_sysOrLocal){
            case sys:
                m_zonedTime = date::make_zoned(Zone, m_sysDays + m_initTime);
                break;
            case local:
                m_zonedTime = date::make_zoned(Zone, m_localDays + m_initTime);
                break;

        }
    }

    void FrTimeZone::Update(double time) {
        auto Zone = date::locate_zone(m_timeZoneName);
        int m_secondes = floor(time);
        int m_milliseconds = floor(1000*(time-floor(time)));
        std::chrono::milliseconds m_actualTime = m_initTime + std::chrono::seconds(m_secondes) + std::chrono::milliseconds(m_milliseconds);
        switch (m_sysOrLocal){
            case sys:
                m_zonedTime = date::make_zoned(Zone, m_sysDays + m_actualTime);
                break;
            case local:
                m_zonedTime = date::make_zoned(Zone, m_localDays + m_actualTime);
                break;

        }
    }


}