//
// Created by Lucas Letournel on 26/09/18.
//

#include <GeographicLib/MagneticModel.hpp>
#include "FrGeographicServices.h"

/**
 * GeographicServices local cartesian is based on NED frame (x=North, y=East, z=Down).
 * In GeographicLib, the local cartesian coordinates is defined as ENU (x=East, y= North, z=Up).
 */

namespace frydom {

    void
    FrGeographicServices::Convert_GeoToCart(const double lat, const double lon, const double h, double &x, double &y,
                                            double &z) {
                m_LocalCartesian->Forward(lat, lon, h, y, x, z);
                z = -z;
    }

    void
    FrGeographicServices::Convert_CartToGeo(const double x, const double y, const double z, double &lat, double &lon,
                                            double &h) {
                m_LocalCartesian->Reverse(y, x, -z, lat, lon, h);
    }

    double FrGeographicServices::ComputeMagneticDeclination(double x, double y, double z, double year) {
        /// Magnetic model loaded from _deps directory
        GeographicLib::MagneticModel magneticModel("emm2017", "../_deps/magneticmodel-src");
        double lat, lon, h;
        /// Convert the node local coordinates to geographical coordinates
        Convert_CartToGeo(x, y, z, lat, lon, h);
        /// Compute the magnetic declination
        double Bx, By, Bz, H, F, D, I;
        magneticModel(year, lat, lon, h, Bx, By, Bz);
        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);
        return D;
    }
}