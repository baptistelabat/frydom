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


#ifndef FRYDOM_FRGEOGRAPHICSERVICES_H
#define FRYDOM_FRGEOGRAPHICSERVICES_H

#include "MathUtils/Vector3d.h"

#include "frydom/core/common/FrConvention.h"
#include "GeographicLib/LocalCartesian.hpp"


namespace frydom {

  //Forward Declaration
  class Position;

  /**
   * \class FrGeographicCoord
   * \brief Class defining geographic coordinates : (latitude, longitude and height).
   */
  class FrGeographicCoord {

   private:
    double m_latitude;
    double m_longitude;
    double m_height;
   public:
    FrGeographicCoord();

    FrGeographicCoord(double lat, double lon, double h);

    explicit FrGeographicCoord(Position geoPos);

    double GetLatitude() const {
      return m_latitude;
    }

    double &GetLatitude() {
      return m_latitude;
    }

    double GetLongitude() const {
      return m_longitude;
    }

    double &GetLongitude() {
      return m_longitude;
    }

    double GetHeight() const {
      return m_height;
    }

    double &GetHeight() {
      return m_height;
    }


  };


  /**
   * \class FrGeographicServices
   * \brief Class to provide convert methods between geographic coordinates and Cartesian positions.
   *
   * FrGeographicServices is a service providing convert methods between geographic coordinates and cartesian positions.
   * It can also compute the magnetic declination of a position given either in geographic or cartesian coordinates.
   * In GeographicServices, the cartesian reference frame depends on the frame convention given (NED/NWU).
   * In GeographicLib, the cartesian coordinates is defined as ENU (x=East, y= North, z=Up).
   *
   */
  class FrGeographicServices {

   private:
    GeographicLib::LocalCartesian m_LocalCartesian; ///> Structure for converting local coordinates to geographic coordinates, contains the geocoord origins

   public:
    /// Default Constructor
    FrGeographicServices();

    /// Get the LocalCartesian variable
    /// \return GeographicLib::LocalCartesian variable
    GeographicLib::LocalCartesian GetGeoLib() const;

    /// Set the geographic origin
    /// \param lat0 latitude of the origin
    /// \param lon0 longitude of the origin
    /// \param h0 height of the origin
    void SetGeographicOrigin(double lat0, double lon0, double h0);

    /// Set the geographic origin
    /// \param geoCoord geographic coordinates
    void SetGeographicOrigin(FrGeographicCoord geoCoord);

    /// Get the geographic position of the origin
    /// \return geographic position of the origin
    FrGeographicCoord GetGeographicOrigin() const;

    /// Get the geographic position of the origin
    /// \param lat latitude of the origin
    /// \param lon longitude of the origin
    /// \param h height of the origin
    void GetGeographicOrigin(double &lat, double &lon, double &h) const;

    //-------------------------GeoToCart-------------------------//
    /// Convert geographic coordinates to cartesian position
    /// \param lat latitude of the geographic coordinates
    /// \param lon longitude of the geographic coordinates
    /// \param h height of the geographic coordinates
    /// \param x x cartesian position in the world reference frame
    /// \param y y cartesian position in the world reference frame
    /// \param z z cartesiant position in the world reference frame
    /// \param fc frame convention (NED/NWU) for the cartesian position
    void GeoToCart(double lat, double lon, double h, double &x, double &y, double &z, FRAME_CONVENTION fc) const;

    /// Convert geographic coordinates to cartesian position
    /// \param lat latitude of the geographic coordinates
    /// \param lon longitude of the geographic coordinates
    /// \param h height of the geographic coordinates
    /// \param fc frame convention (NED/NWU) for the cartesian position
    /// \return cartesian position
    Position GeoToCart(double lat, double lon, double h, FRAME_CONVENTION fc) const;

    /// Convert geographic coordinates to cartesian position
    /// \param geoCoord geographic coordinates
    /// \param fc frame convention (NED/NWU)
    /// \return cartesian position in the world reference frame
    Position GeoToCart(const FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const;

    /// Convert geographic coordinates to cartesian position
    /// \param geoCoord geographic coordinates
    /// \param cartPos cartesian position in the world reference frame
    /// \param fc frame convention (NED/NWU) for the cartesian position
    void GeoToCart(const FrGeographicCoord &geoCoord, Position &cartPos, FRAME_CONVENTION fc) const;


    //-------------------------CartToGeo-------------------------//
    /// Convert cartesian position to geographic coordinates
    /// \param x x cartesian position in the world reference frame
    /// \param y y cartesian position in the world reference frame
    /// \param z z cartesiant position in the world reference frame
    /// \param lat latitude of the geographic coordinates
    /// \param lon longitude of the geographic coordinates
    /// \param h height of the geographic coordinates
    /// \param fc frame convention (NED/NWU) for the cartesian position
    void CartToGeo(double x, double y, double z, double &lat, double &lon, double &h, FRAME_CONVENTION fc) const;

    /// Convert cartesian position to geographic coordinates
    /// \param cartPos cartesian position in the world reference frame
    /// \param geoCoord geographic coordinates
    /// \param fc frame convention (NED/NWU) for the cartesian position
    void CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const;

    /// Convert cartesian position to geographic coordinates
    /// \param cartPos cartesian position in the world reference frame
    /// \param fc frame convention (NED/NWU) for the cartesian position
    /// \return geographic coordinates
    FrGeographicCoord CartToGeo(const Position &cartPos, FRAME_CONVENTION fc) const;


    //-------------------------Magnetic Declination-------------------------//
    /// Compute the magnetic declination of a cartesian position
    /// \param cartPos cartesian position in the world reference frame
    /// \param year year
    /// \param fc frame convention (NED/NWU) for the cartesian position
    /// \return magnetic declination
    double GetDeclinationFromCart(const Position &cartPos, double year, FRAME_CONVENTION fc) const;

    /// Compute the magnetic declination of a cartesian position
    /// \param x x cartesian position in the world reference frame
    /// \param y y cartesian position in the world reference frame
    /// \param z z cartesiant position in the world reference frame
    /// \param year year
    /// \param fc frame convention (NED/NWU) for the cartesian position
    /// \return magnetic declination
    double GetDeclinationFromCart(double x, double y, double z, double year, FRAME_CONVENTION fc) const;

    /// Compute the magnetic declination of a geographic coordinates
    /// \param geoCoord geographic coordinates
    /// \param year year
    /// \return magnetic declination
    double GetDeclinationFromGeo(const FrGeographicCoord &geoCoord, double year) const;

    /// Compute the magnetic declination of a geographic coordinates
    /// \param lat latitude of the geographic coordinates
    /// \param lon longitude of the geographic coordinates
    /// \param h height of the geographic coordinates
    /// \param year year
    /// \return magnetic declination
    double GetDeclinationFromGeo(double lat, double lon, double h, double year) const;

    /// Get the magnetic field components, in nanotesla, at the geographic coordinates
    /// \param geoCoord Geographic coordinates on which to get the magnetic field
    /// \param year year for chich to get the magnetic field
    /// \param fc frame convention (NED/NWU) for the magnetic field vector
    /// \return magnetic field components, in nanotesla
    mathutils::Vector3d<double>
    GetMagneticComponentsFromGeo(const FrGeographicCoord &geoCoord, double year, FRAME_CONVENTION fc) const;


    /// Get the magnetic field components, in nanotesla, at a cartesian position
    /// \param cartPos cartesian position on which to get the magnetic field
    /// \param year year for chich to get the magnetic field
    /// \param fc frame convention (NED/NWU) for the cartesian position and magnetic field vector
    /// \return magnetic field components, in nanotesla
    mathutils::Vector3d<double>
    GetMagneticComponentsFromCart(const Position &cartPos, double year, FRAME_CONVENTION fc) const;

  };


} // end namespace frydom


#endif //FRYDOM_FRGEOGRAPHICSERVICES_H
