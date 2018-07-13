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

    double FrEnvironment::ComputeMagneticDeclination(const chrono::ChVector<> localPos) {
        /// Magnetic model loaded from _deps directory
        GeographicLib::MagneticModel magneticModel("emm2017","../_deps/magneticmodel-src");
        double lat, lon, h;
        /// Convert the node local coordinates to geographical coordinates
        Convert_CartToGeo(localPos.x(),localPos.y(),localPos.z(),lat,lon,h);
        /// Get the UTC time to obtain the year
        auto lt = GetTimeZone()->GetUTCTime();
        date::year_month_day ymd{date::floor<date::days>(lt)};
        double myYear = int(ymd.year());
        /// Compute the magnetic declination
        double Bx, By, Bz, H, F, D, I;
        magneticModel(myYear,lat,lon,h,Bx,By,Bz);
        magneticModel.FieldComponents(Bx,By,Bz,H,F,D,I);
        return D;
    }
}