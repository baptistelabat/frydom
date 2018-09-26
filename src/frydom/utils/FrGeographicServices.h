//
// Created by Lucas Letournel on 26/09/18.
//

#ifndef FRYDOM_FRGEOGRAPHICSERVICES_H
#define FRYDOM_FRGEOGRAPHICSERVICES_H

// GeographicLib includes
#include "GeographicLib/LocalCartesian.hpp"

#include "frydom/core/FrConstants.h"

/** The local cartesian coordinates defined in FRyDoM are based on either NWU (x=North, y=West, z=Up),
 * or NED (x=North, y=East, z=Down) configurations. In GeographicLib, the local cartesian coordinates is defined as
 * (x=East, y= North, z=Up).
 */

namespace frydom {
    class FrGeographicServices {
    private:
        /// Structure for converting local coordinates to geographic coordinates, contains the geocoord origins
        std::unique_ptr<GeographicLib::LocalCartesian> m_LocalCartesian;

    public:
        ///
        FrGeographicServices(){
        m_LocalCartesian = std::make_unique<GeographicLib::LocalCartesian>();}
        
        /// Getter of the LocalCartesian variable
        GeographicLib::LocalCartesian *GetGeoLib() const {
            return m_LocalCartesian.get();
        }

        /// Setter of the LocalCartesian variable
        void SetGeographicOrigin(const double lat0, const double lon0, const double h0) {
            m_LocalCartesian->Reset(lat0, lon0, h0);
        }

        /// Convert from geographic to local coordinates
        void Convert_GeoToCart(double lat, double lon, double h, double &x, double &y, double &z, FrFrame frame = NWU);

        /// Convert from local to geographic coordinates
        void Convert_CartToGeo(double x, double y, double z, double &lat, double &lon, double &h, FrFrame frame = NWU);

        /// Compute the magnetic declination for a cartesian position
        double ComputeMagneticDeclination(double x, double y, double z, double year, FrFrame frame = NWU);

    };

}
#endif //FRYDOM_FRGEOGRAPHICSERVICES_H
