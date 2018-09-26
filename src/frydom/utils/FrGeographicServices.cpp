//
// Created by Lucas Letournel on 26/09/18.
//

#include <GeographicLib/MagneticModel.hpp>
#include "FrGeographicServices.h"

/** The local cartesian coordinates defined in FRyDoM are based on either NWU (x=North, y=West, z=Up),
 * or NED (x=North, y=East, z=Down) configurations. In GeographicLib, the local cartesian coordinates is defined as
 * (x=East, y= North, z=Up).
 */

namespace frydom {

    void
    FrGeographicServices::Convert_GeoToCart(const double lat, const double lon, const double h, double &x, double &y,
                                            double &z, FrFrame frame) {
        switch (frame) {
            case NWU:
                m_LocalCartesian->Forward(lat, lon, h, y, x, z);
                y = -y;
                break;
            case NED:
                m_LocalCartesian->Forward(lat, lon, h, y, x, z);
                z = -z;
                break;
        }
    }

    void
    FrGeographicServices::Convert_CartToGeo(const double x, const double y, const double z, double &lat, double &lon,
                                            double &h, FrFrame frame) {
        switch (frame) {
            case NWU:
                m_LocalCartesian->Reverse(-y, x, z, lat, lon, h);
            break;
            case NED:
                m_LocalCartesian->Reverse(y, x, -z, lat, lon, h);

            break;
        }
    }

    double FrGeographicServices::ComputeMagneticDeclination(double x, double y, double z, double year, FrFrame frame) {
        /// Magnetic model loaded from _deps directory
        GeographicLib::MagneticModel magneticModel("emm2017", "../_deps/magneticmodel-src");
        double lat, lon, h;
        /// Convert the node local coordinates to geographical coordinates
        Convert_CartToGeo(x, y, z, lat, lon, h, frame);
        /// Compute the magnetic declination
        double Bx, By, Bz, H, F, D, I;
        magneticModel(year, lat, lon, h, Bx, By, Bz);
        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);
        return D;
    }
}