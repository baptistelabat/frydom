//
// Created by Lucas Letournel on 26/09/18.
//

#ifndef FRYDOM_FRGEOGRAPHICSERVICES_H
#define FRYDOM_FRGEOGRAPHICSERVICES_H

#include <memory>

// GeographicLib includes
//#include "GeographicLib/LocalCartesian.hpp"


/**
 * GeographicServices local cartesian is based on NED frame (x=North, y=East, z=Down).
 * In GeographicLib, the local cartesian coordinates is defined as ENU (x=East, y= North, z=Up).
 */

namespace GeographicLib {
    class LocalCartesian;
}

namespace frydom {

    class FrGeographicServices {

    private:
        /// Structure for converting local coordinates to geographic coordinates, contains the geocoord origins
        std::unique_ptr<GeographicLib::LocalCartesian> m_LocalCartesian;

    public:
        ///
        FrGeographicServices();

        /// Getter of the LocalCartesian variable
        GeographicLib::LocalCartesian *GetGeoLib() const;

        /// Setter of the LocalCartesian variable
        void SetGeographicOrigin(const double lat0, const double lon0, const double h0);

        /// Convert from geographic to local coordinates
        void Convert_GeoToCart(double lat, double lon, double h, double &x_north, double &y_east, double &z_down);

        /// Convert from local to geographic coordinates
        void Convert_CartToGeo(double x_north, double y_east, double z_down, double &lat, double &lon, double &h);

        /// Compute the magnetic declination for a cartesian position
        double ComputeMagneticDeclination(double x_north, double y_east, double z_down, double year);

    };

}
#endif //FRYDOM_FRGEOGRAPHICSERVICES_H
