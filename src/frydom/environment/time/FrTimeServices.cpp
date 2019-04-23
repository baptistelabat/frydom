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


#include "FrTimeServices.h"
#include <cmath>
#include <iostream>

namespace frydom {


    FrTimeServices::FrTimeServices() {

        auto tp = date::make_zoned(date::current_zone(), std::chrono::system_clock::now());
        auto lt = tp.get_local_time();
        m_initDay = date::floor<date::days>(lt);
        m_initTime = date::floor<std::chrono::milliseconds>(lt - m_initDay);

    }

//    void FrTimeServices::SetStartingTime(int Hours, int Minutes, int Seconds) {
//        m_time = std::chrono::hours(Hours) + std::chrono::minutes(Minutes) + std::chrono::seconds(Seconds);
//    }

    void FrTimeServices::SetStartingTime(std::chrono::hours Hours, std::chrono::minutes Minutes, std::chrono::seconds Seconds) {
        m_initTime = Hours + Minutes + Seconds;
    }

    void FrTimeServices::SetStartingTime(std::chrono::milliseconds time) {
        m_initTime = time;
    }

    void FrTimeServices::SetStartingTime(date::zoned_time<std::chrono::milliseconds> time) {
        auto lt = time.get_local_time();
        m_initDay = date::floor<date::days>(lt);
        m_initTime = date::floor<std::chrono::milliseconds>(lt - m_initDay);

    }

    void FrTimeServices::SetStartingDay(date::local_days Day) {
        m_initDay = Day;
    }

    void FrTimeServices::SetStartingDay(date::year Year, date::month Month, date::day Day) {
        m_initDay = date::local_days{Year/Month/Day};
    }

    void FrTimeServices::Update(double time) {

        int m_secondes = static_cast<int>(floor(time));
        int m_milliseconds = static_cast<int>(floor(1000 * (time - floor(time))));
        c_time = m_initTime + std::chrono::seconds(m_secondes) + std::chrono::milliseconds(m_milliseconds);

    }

    double FrTimeServices::GetTimeZoneOffset(const date::time_zone* timeZone) {
        auto UTC = date::locate_zone("UTC");
        auto UTCTime = date::make_zoned(UTC, m_initDay + c_time).get_sys_time();
        return timeZone->get_info(UTCTime).offset.count()/60.;
    }

    double FrTimeServices::GetTimeZoneOffset(const std::string &timeZoneName) {
        //TODO: générer erreur si mauvais nom de timezone
        auto timeZone = date::locate_zone(timeZoneName);
        return GetTimeZoneOffset(timeZone);
    }

    void FrTimeServices::Initialize() {
        c_time = m_initTime;
    }


}
