//
// Created by Letournel on 02/06/18.
//
//#include "date/tz.h"
#include <chrono>
#include <iostream>
#include <date/tz.h>
#include "fmt/format.h"

#include "frydom/environment/FrTimeZone.h"

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

    auto tod = make_time(2h + 16min + 45s + 23ms);
    std::cout << tod << '\n';
    auto d = milliseconds(tod);
    std::cout << d << '\n';
    auto todoo = make_time(d);
    std::cout << todoo << '\n';;

    auto truc = t_New_York.get_local_time();
    std::cout << truc << '\n';
    truc += d;
    std::cout << truc << '\n';

    auto ymd = jul/29/2018;
    auto dp = sys_days{ymd};
    auto tp = dp + d;
    std::cout << tp << '\n';

    auto tpp = make_zoned(zone,tp);
    std::cout << tpp << '\n';


    /*typedef std::chrono::duration<int,std::ratio<3600*24>> days;
    auto daypoint = floor<days>(t_New_York);
    auto ymd = year_month_day(daypoint);   // calendar date
    auto tod = make_time(t_New_York - daypoint); // Yields time_of_day type

// Obtain individual components as integers
    auto y   = int(ymd.year());
    auto m   = unsigned(ymd.month());
    auto d   = unsigned(ymd.day());
    auto h   = tod.hours().count();
    auto min = tod.minutes().count();
    auto s   = tod.seconds().count();

    fmt::print("{}y,{}m,{}d : {}h,{}min,{}s",y,m,d,h,min,s);*/


    double t_simu = 10.8; // Simulation time in seconds
    int seco = 10, mseco=800;

    auto dur = seconds(seco);
    auto dur2 = milliseconds(mseco);

    milliseconds test;
    test = dur + dur2;
    std::cout << test << '\n';



    /*

    seconds sec(1);
    std::cout << sec.count() << std::endl;
    typedef duration<double> truc;
    std::cout << std::chrono::duration_cast<truc>(sec).count()
                                                    << " truc\n";

    auto t_test = 10s;
    auto t_updated = make_zoned(zone,t_here.get_sys_time() + t_test);
    std::cout << t_updated << '\n';

    time_point <system_clock, duration<long>> today_is;
    today_is = date::sys_days{jun / 15 / 2018} + 15h + 59min + 59s; //date::local_days{Monday[1]/May/2016} + 9h
    auto meet_nyc = make_zoned("America/New_York", today_is);
    std::cout << "The New York meeting is " << meet_nyc << '\n';*/

    return 0;
}
