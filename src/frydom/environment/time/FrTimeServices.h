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

    class FrTimeServices {

    private:
        std::chrono::milliseconds c_time;

        std::chrono::milliseconds m_initTime;

        date::local_days m_initDay;

    public:

        FrTimeServices();

        ~FrTimeServices() = default;

//        void SetStartingDay(int Year, int Month, int Day);

        void SetStartingDay(date::local_days Day);

        void SetStartingDay(date::year Year, date::month Month, date::day Day);

//        void SetStartingTime(int Hours, int Minutes, int Seconds);

        void SetStartingTime(std::chrono::hours Hours, std::chrono::minutes Minutes, std::chrono::seconds Seconds);

        void SetStartingTime(std::chrono::milliseconds time);


        auto GetTime(const date::time_zone* timeZone = date::current_zone()) const {
            return date::make_zoned(timeZone, m_initDay + c_time).get_local_time();
        }

        auto GetTime(const std::string &timeZoneName) const {
            //TODO: générer erreur si mauvais nom de timezone
            auto timeZone = date::locate_zone(timeZoneName);
            return GetTime(timeZone);
        }

        auto GetUTCTime() const {
            auto UTC = date::locate_zone("UTC");
            return GetTime(UTC);
        }

        double GetTimeZoneOffset(const std::string &timeZoneName); // returns time zone offset in minutes

        double GetTimeZoneOffset(const date::time_zone* timeZone); // returns time zone offset in minutes

        void Initialize();

        void Update(double time);

    };

}
#endif //FRYDOM_FRTIMESERVICES_H
