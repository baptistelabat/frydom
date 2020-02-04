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


#include "FrGeographicServices.h"

#include "GeographicLib/MagneticModel.hpp"
#include "frydom/core/math/FrVector.h"


/*
 * In GeographicServices, the cartesian reference frame depends on the frame convention given (NED/NWU).
 * In GeographicLib, the cartesian coordinates is defined as ENU (x=East, y= North, z=Up).
 */

namespace frydom {

  FrGeographicCoord::FrGeographicCoord(const Position geoPos) {
    m_latitude = geoPos.GetX();
    m_longitude = geoPos.GetY();
    m_height = geoPos.GetZ();
  }

  FrGeographicCoord::FrGeographicCoord() {
    m_latitude = 0.;
    m_longitude = 0.;
    m_height = 0.;
  }

  FrGeographicCoord::FrGeographicCoord(double lat, double lon, double h) {
    m_latitude = lat;
    m_longitude = lon;
    m_height = h;
  }

  FrGeographicServices::FrGeographicServices() {
    GeographicLib::LocalCartesian m_LocalCartesian();
  }

  GeographicLib::LocalCartesian FrGeographicServices::GetGeoLib() const {
    return m_LocalCartesian;
  }

  void FrGeographicServices::SetGeographicOrigin(const double lat0, const double lon0, const double h0) {
    m_LocalCartesian.Reset(lat0, lon0, h0);
  }

  void FrGeographicServices::SetGeographicOrigin(const FrGeographicCoord geoCoord) {
    SetGeographicOrigin(geoCoord.GetLatitude(), geoCoord.GetLongitude(), geoCoord.GetHeight());
  }

  FrGeographicCoord FrGeographicServices::GetGeographicOrigin() const {
    return {m_LocalCartesian.LatitudeOrigin(), m_LocalCartesian.LongitudeOrigin(), m_LocalCartesian.HeightOrigin()};
  }

  void FrGeographicServices::GetGeographicOrigin(double &lat, double &lon, double &h) const {
    lat = m_LocalCartesian.LatitudeOrigin();
    lon = m_LocalCartesian.LongitudeOrigin();
    h = m_LocalCartesian.HeightOrigin();
  }


  void FrGeographicServices::GeoToCart(double lat, double lon, double h,
                                       double &x, double &y, double &z, FRAME_CONVENTION fc) const {
    m_LocalCartesian.Forward(lat, lon, h, y, x, z);
    if (IsNED(fc)) z = -z;
    else y = -y;
  }

  Position FrGeographicServices::GeoToCart(const FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const {
    double x, y, z;
    GeoToCart(geoCoord.GetLatitude(), geoCoord.GetLongitude(), geoCoord.GetHeight(), x, y, z, fc);
    return {x, y, z};
  }

  Position FrGeographicServices::GeoToCart(double lat, double lon, double h, FRAME_CONVENTION fc) const {
    return GeoToCart(FrGeographicCoord(lat, lon, h), fc);
  }

  void
  FrGeographicServices::GeoToCart(const FrGeographicCoord &geoCoord, Position &cartPos, FRAME_CONVENTION fc) const {
    cartPos = GeoToCart(geoCoord, fc);
  }


  void FrGeographicServices::CartToGeo(double x, double y, double z,
                                       double &lat, double &lon, double &h, FRAME_CONVENTION fc) const {
    double Ytemp = y, Ztemp = z;
    if (IsNED(fc)) Ztemp = -Ztemp;
    else Ytemp = -Ytemp;
    m_LocalCartesian.Reverse(Ytemp, x, Ztemp, lat, lon, h);
  }

  FrGeographicCoord FrGeographicServices::CartToGeo(const Position &cartPos, FRAME_CONVENTION fc) const {
    double lat, lon, h;
    CartToGeo(cartPos.GetX(), cartPos.GetY(), cartPos.GetZ(), lat, lon, h, fc);
    return {lat, lon, h};
  }

  void
  FrGeographicServices::CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const {
    geoCoord = CartToGeo(cartPos, fc);
  }


  double FrGeographicServices::GetDeclinationFromCart(const Position &cartPos, double year, FRAME_CONVENTION fc) const {
    /// Convert the node local coordinates to geographical coordinates
    auto geoCoord = CartToGeo(cartPos, fc);

    return GetDeclinationFromGeo(geoCoord, year);
  }

  double FrGeographicServices::GetDeclinationFromCart(double x, double y, double z,
                                                      double year, FRAME_CONVENTION fc) const {
    return GetDeclinationFromCart(Position(x, y, z), year, fc);
  }

  double
  FrGeographicServices::GetDeclinationFromGeo(const FrGeographicCoord &geoCoord, double year) const {

    // Compute the magnetic declination
    mathutils::Vector3d<double> magComponent;
    magComponent = GetMagneticComponentsFromGeo(geoCoord, year, NWU);

    double H, F, D, I;
    GeographicLib::MagneticModel::FieldComponents(-magComponent.y(), magComponent.x(), magComponent.z(), H, F, D, I);

    return D;
  }

  double
  FrGeographicServices::GetDeclinationFromGeo(double lat, double lon, double h, double year) const {
    return GetDeclinationFromGeo(FrGeographicCoord(lat, lon, h), year);
  }

  mathutils::Vector3d<double>
  FrGeographicServices::GetMagneticComponentsFromGeo(const FrGeographicCoord &geoCoord, double year,
                                                     FRAME_CONVENTION fc) const {

    // Magnetic model loaded from _deps directory
    // GEOGRAPHICLIB_MAGNETIC_PATH is a compilation variable, defined in Add_GeographicLib.cmake, for GeographicLib target only
    GeographicLib::MagneticModel magneticModel("emm2017", GEOGRAPHICLIB_MAGNETIC_PATH);

    // Compute the magnetic declination
    double Bx, By, Bz;
    magneticModel(year, geoCoord.GetLatitude(), geoCoord.GetLongitude(), geoCoord.GetHeight(), Bx, By, Bz);

    // return the vector in the specified frame convention
    mathutils::Vector3d<double> vecNWU = {By, -Bx, Bz};
    if (IsNED(fc)) internal::SwapFrameConvention(vecNWU);

    return vecNWU;

  }

  mathutils::Vector3d<double>
  FrGeographicServices::GetMagneticComponentsFromCart(const Position &cartPos, double year, FRAME_CONVENTION fc) const {
    auto geoCoord = CartToGeo(cartPos, fc);
    return GetMagneticComponentsFromGeo(geoCoord, year, fc);
  }


}  // end namespace frydom
