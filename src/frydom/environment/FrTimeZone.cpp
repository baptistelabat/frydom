//
// Created by Lucas Letournel on 15/06/18.
//

#include "FrTimeZone.h"

namespace frydom {
    std::tm FrTimeZone::to_tm(date::zoned_seconds tp) {
        /// FROM https://github.com/HowardHinnant/date/wiki/Examples-and-Recipes#converting-to-a-tm
        /*using namespace date;
        using namespace std;
        using namespace std::chrono;*/
        auto lt = tp.get_local_time();
        auto ld = floor<days>(lt);
        time_of_day<seconds> tod{lt - ld};  // <seconds> can be omitted in C++17
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
        t.tm_isdst = tp.get_info().save != minutes{0};
        return t;

        return tm();
    }

    date::sys_seconds FrTimeZone::to_sys_time(std::tm const &t) {
        //using namespace date;
        //using namespace std::chrono;

        //return date::sys_days{std::chrono::year{t.tm_year + 1900} / (t.tm_mon + 1) / t.tm_mday}
        //       + std::chrono::hours{t.tm_hour} + std::chrono::minutes{t.tm_min} + std::chrono::seconds{t.tm_sec};
    }

    date::local_seconds FrTimeZone::to_local_time(std::tm const &t) {
        //using namespace date;
        //using namespace std::chrono;
        /*return date::local_days{std::chrono::year{t.tm_year + 1900} / (t.tm_mon + 1) / t.tm_mday}
               + std::chrono::hours{t.tm_hour} + std::chrono::minutes{t.tm_min} + std::chrono::seconds{t.tm_sec};*/
    }
}