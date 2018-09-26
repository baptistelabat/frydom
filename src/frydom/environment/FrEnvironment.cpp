//
// Created by frongere on 10/10/17.
//

#include <GeographicLib/MagneticModel.hpp>
#include "FrEnvironment.h"
#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {


    void FrEnvironment::SetFreeSurface(FrFreeSurface *freeSurface) {
        m_freeSurface = std::unique_ptr<FrFreeSurface>(freeSurface);
        m_system->AddBody(m_freeSurface->GetBody());
    }



    int FrEnvironment::GetYear() {
        /// Get the UTC time to obtain the year
        auto lt = GetTimeZone()->GetUTCTime();
        date::year_month_day ymd{date::floor<date::days>(lt)};
        return int(ymd.year());
    }
}