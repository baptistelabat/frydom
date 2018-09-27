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

    double FrEnvironment::ComputeMagneticDeclination(double x, double y, double z) {
        /// Magnetic model loaded from _deps directory
        GeographicLib::MagneticModel magneticModel("emm2017", "../_deps/magneticmodel-src");
        double lat, lon, h;
        /// Convert the node local coordinates to geographical coordinates
        Convert_CartToGeo(x, y, z, lat, lon, h);
        /// Compute the magnetic declination
        double Bx, By, Bz, H, F, D, I;
        magneticModel(GetYear(), lat, lon, h, Bx, By, Bz);
        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);
        return D;
    }
}