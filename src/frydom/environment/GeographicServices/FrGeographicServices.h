//
// Created by Lucas Letournel on 26/09/18.
//

#ifndef FRYDOM_FRGEOGRAPHICSERVICES_H
#define FRYDOM_FRGEOGRAPHICSERVICES_H

#include <memory>
#include "frydom/core/FrGeographic.h"

// GeographicLib includes
#include "GeographicLib/LocalCartesian.hpp"


/**
 * GeographicServices is a service providing convert methods between geographic coordinates and cartesian positions.
 * It can also compute the magnetic declination of a position given either in geographic or cartesian coordinates.
 * In GeographicServices, the cartesian reference frame depends on the frame convention given (NED/NWU).
 * In GeographicLib, the cartesian coordinates is defined as ENU (x=East, y= North, z=Up).
 */

namespace GeographicLib {
    class LocalCartesian;
}

namespace frydom {

    //Forward Declaration
    class Position;

    /// Class defining geographic coordinates : (latitude, longitude and elevation).
    class FrGeographicCoord {

    private:
        double m_latitude;
        double m_longitude;
        double m_elevation;
    public:
        FrGeographicCoord();
        FrGeographicCoord(double lat, double lon, double h);
        explicit FrGeographicCoord(Position geoPos);

        double GetLatitude() const {
                return m_latitude;
        }
        double& GetLatitude(){
                return m_latitude;
        }
        double GetLongitude() const {
                return m_longitude;
        }
        double& GetLongitude(){
                return m_longitude;
        }
        double GetElevation() const{
                return m_elevation;
        }
        double& GetElevation(){
                return m_elevation;
        }


    };


    class FrGeographicServices {

    private:
        /// Structure for converting local coordinates to geographic coordinates, contains the geocoord origins
        GeographicLib::LocalCartesian m_LocalCartesian;

    public:
        /// Default Constructor
        FrGeographicServices();

        /// Get the LocalCartesian variable
        /// \return GeographicLib::LocalCartesian variable
        GeographicLib::LocalCartesian GetGeoLib() const;

        /// Set the geographic origin
        /// \param lat0 latitude of the origin
        /// \param lon0 longitude of the origin
        /// \param h0 elevation of the origin
        void SetGeographicOrigin(double lat0, double lon0, double h0);

        //-------------------------GeoToCart-------------------------//
        /// Convert geographic coordinates to cartesian position
        /// \param lat latitude of the geographic coordinates
        /// \param lon longitude of the geographic coordinates
        /// \param h elevation of the geographic coordinates
        /// \param x x cartesian position in the world reference frame
        /// \param y y cartesian position in the world reference frame
        /// \param z z cartesiant position in the world reference frame
        /// \param fc frame convention (NED/NWU) for the cartesian position
        void GeoToCart(double lat, double lon, double h, double &x, double &y, double &z, FRAME_CONVENTION fc);

        /// Convert geographic coordinates to cartesian position
        /// \param lat latitude of the geographic coordinates
        /// \param lon longitude of the geographic coordinates
        /// \param h elevation of the geographic coordinates
        /// \param fc frame convention (NED/NWU) for the cartesian position
        /// \return cartesian position
        Position GeoToCart(double lat, double lon, double h, FRAME_CONVENTION fc);

        /// Convert geographic coordinates to cartesian position
        /// \param geoCoord geographic coordinates
        /// \param fc frame convention (NED/NWU)
        /// \return cartesian position in the world reference frame
        Position GeoToCart(const FrGeographicCoord& geoCoord, FRAME_CONVENTION fc);

        /// Convert geographic coordinates to cartesian position
        /// \param geoCoord geographic coordinates
        /// \param cartPos cartesian position in the world reference frame
        /// \param fc frame convention (NED/NWU) for the cartesian position
        void GeoToCart(const FrGeographicCoord& geoCoord, Position& cartPos, FRAME_CONVENTION fc);


        //-------------------------CartToGeo-------------------------//
        /// Convert cartesian position to geographic coordinates
        /// \param x x cartesian position in the world reference frame
        /// \param y y cartesian position in the world reference frame
        /// \param z z cartesiant position in the world reference frame
        /// \param lat latitude of the geographic coordinates
        /// \param lon longitude of the geographic coordinates
        /// \param h elevation of the geographic coordinates
        /// \param fc frame convention (NED/NWU) for the cartesian position
        void CartToGeo(double x, double y, double z, double &lat, double &lon, double &h,
                       FRAME_CONVENTION fc);

        /// Convert cartesian position to geographic coordinates
        /// \param cartPos cartesian position in the world reference frame
        /// \param geoCoord geographic coordinates
        /// \param fc frame convention (NED/NWU) for the cartesian position
        void CartToGeo(const Position& cartPos, FrGeographicCoord& geoCoord, FRAME_CONVENTION fc);

        /// Convert cartesian position to geographic coordinates
        /// \param cartPos cartesian position in the world reference frame
        /// \param fc frame convention (NED/NWU) for the cartesian position
        /// \return geographic coordinates
        FrGeographicCoord CartToGeo(const Position& cartPos, FRAME_CONVENTION fc);


        //-------------------------Magnetic Declination-------------------------//
        /// Compute the magnetic declination of a cartesian position
        /// \param cartPos cartesian position in the world reference frame
        /// \param year year
        /// \param fc frame convention (NED/NWU) for the cartesian position
        /// \return magnetic declination
        double GetDeclinationFromCart(const Position &cartPos, double year, FRAME_CONVENTION fc);

        /// Compute the magnetic declination of a cartesian position
        /// \param x x cartesian position in the world reference frame
        /// \param y y cartesian position in the world reference frame
        /// \param z z cartesiant position in the world reference frame
        /// \param year year
        /// \param fc frame convention (NED/NWU) for the cartesian position
        /// \return magnetic declination
        double GetDeclinationFromCart(double x, double y, double z, double year, FRAME_CONVENTION fc);

        /// Compute the magnetic declination of a geographic coordinates
        /// \param geoCoord geographic coordinates
        /// \param year year
        /// \return magnetic declination
        double GetDeclinationFromGeo(const FrGeographicCoord& geoCoord, double year);

        /// Compute the magnetic declination of a geographic coordinates
        /// \param lat latitude of the geographic coordinates
        /// \param lon longitude of the geographic coordinates
        /// \param h elevation of the geographic coordinates
        /// \param year year
        /// \return magnetic declination
        double GetDeclinationFromGeo(double lat, double lon, double h, double year);

    };

}
#endif //FRYDOM_FRGEOGRAPHICSERVICES_H
