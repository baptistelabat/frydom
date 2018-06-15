//
// Created by Lucas Letournel on 15/06/18.
//

#ifndef FRYDOM_FRTIMEZONE_H
#define FRYDOM_FRTIMEZONE_H

#include <chrono>
#include "include/date/tz.h"

using namespace std::chrono;
using namespace date;

class FrTimeZone {
private:
    time_zone * m_timeZone;
    zoned_time <duration<long>> m_zonedTime;
public:
    void now() {std::chrono::system_clock::now();}
    void current_zone() {date::current_zone();}

    auto GetTimeZone() {return m_timeZone;}
    std::string GetName() {return m_timeZone->name();}

    auto GetZonedTime() {return m_zonedTime;}
    auto GetLocalTime() {return m_zonedTime.get_local_time();}
    auto GetSysTime() { return m_zonedTime.get_sys_time();}

};


#endif //FRYDOM_FRTIMEZONE_H
