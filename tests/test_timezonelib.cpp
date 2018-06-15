//
// Created by Letournel on 02/06/18.
//
//#include "date/tz.h"
#include <chrono>
#include <iostream>
#include <include/date/tz.h>
//#include "fmt/format.h"

/// List of tz database time zones : https://en.wikipedia.org/wiki/List_of_tz_database_time_zones

int main() {
    using namespace date;
    using namespace std::chrono;
    using namespace std::chrono_literals;
    auto t_here = make_zoned(current_zone(), system_clock::now());
    std::cout << t_here << '\n';

    auto zone = locate_zone("America/New_York");
    auto t_New_York = make_zoned(zone, system_clock::now());
    std::cout << t_New_York << '\n';

    double t_simu; // Simulation time in seconds
    t_simu = 10;

    auto t_test = seconds(t_simu);
    auto t_updated = make_zoned(zone,t_here.get_sys_time() + t_test);
    std::cout << t_updated << '\n';

    auto test = zone;




    return 0;
}
