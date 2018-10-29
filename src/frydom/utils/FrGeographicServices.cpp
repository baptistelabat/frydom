//
// Created by Lucas Letournel on 26/09/18.
//

#include "FrGeographicServices.h"

#include "GeographicLib/LocalCartesian.hpp"
#include <GeographicLib/MagneticModel.hpp>


/**
 * GeographicServices local cartesian frame is based on NED frame (x=North, y=East, z=Down).
 * In GeographicLib, the local cartesian frame is defined as ENU (x=East, y= North, z=Up).
 */

namespace frydom {

    void FrGeographicServices::Convert_GeoToCart(const double lat, const double lon, const double h,
                                            double &x_north, double &y_east, double &z_down) {
        double z_up;
        m_LocalCartesian->Forward(lat, lon, h, y_east, x_north, z_up);
        z_down = -z_up;
    }

    void FrGeographicServices::Convert_CartToGeo(const double x_north, const double y_east, const double z_down,
                                            double &lat, double &lon, double &h) {
        m_LocalCartesian->Reverse(y_east, x_north, -z_down, lat, lon, h);
    }

    double FrGeographicServices::ComputeMagneticDeclination(double x_north, double y_east, double z_down, double year) {

        /// Magnetic model loaded from _deps directory
        GeographicLib::MagneticModel magneticModel("emm2017", "../_deps/magneticmodel-src");
        double lat, lon, h;

        /// Convert the node local coordinates to geographical coordinates
        Convert_CartToGeo(x_north, y_east, z_down, lat, lon, h);

        /// Compute the magnetic declination
        double Bx, By, Bz, H, F, D, I;
        magneticModel(year, lat, lon, h, Bx, By, Bz);
        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);

        return D;
    }






    FrGeographicServices::FrGeographicServices() {
        m_LocalCartesian = std::make_unique<GeographicLib::LocalCartesian>();
    }

    GeographicLib::LocalCartesian *FrGeographicServices::GetGeoLib() const {
        return m_LocalCartesian.get();
    }

    void FrGeographicServices::SetGeographicOrigin(const double lat0, const double lon0, const double h0) {
        m_LocalCartesian->Reset(lat0, lon0, h0);
    }
}