// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#include "FrGeographicServices.h"

#include "GeographicLib/LocalCartesian.hpp"
#include <GeographicLib/MagneticModel.hpp>
#include <frydom/core/math/FrVector.h>


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

    void FrGeographicServices::GetGeographicOrigin(double& lat, double& lon, double& h) const {
        lat = m_LocalCartesian.LatitudeOrigin();
        lon = m_LocalCartesian.LongitudeOrigin();
        h = m_LocalCartesian.HeightOrigin();
    }



    void FrGeographicServices::GeoToCart(double lat, double lon, double h,
                                         double &x, double &y, double &z, FRAME_CONVENTION fc) const {
        m_LocalCartesian.Forward(lat, lon, h, y, x, z);
        if (IsNED(fc)) z= -z;
        else y = -y;
    }

    Position FrGeographicServices::GeoToCart(const FrGeographicCoord& geoCoord, FRAME_CONVENTION fc) const {
        double x,y,z;
        GeoToCart(geoCoord.GetLatitude(), geoCoord.GetLongitude(), geoCoord.GetHeight(), x, y, z, fc);
        return {x,y,z};
    }

    Position FrGeographicServices::GeoToCart(double lat, double lon, double h, FRAME_CONVENTION fc) const {
        return GeoToCart(FrGeographicCoord(lat,lon,h),fc);
    }

    void FrGeographicServices::GeoToCart(const FrGeographicCoord& geoCoord, Position& cartPos, FRAME_CONVENTION fc) const {
        cartPos = GeoToCart(geoCoord,fc);
    }


    void FrGeographicServices::CartToGeo(double x, double y, double z,
                                         double &lat, double &lon, double &h, FRAME_CONVENTION fc) const {
        double Ytemp = y, Ztemp = z;
        if (IsNED(fc)) Ztemp= -Ztemp;
        else Ytemp = -Ytemp;
        m_LocalCartesian.Reverse(Ytemp, x, Ztemp, lat, lon, h);
    }

    FrGeographicCoord FrGeographicServices::CartToGeo(const Position& cartPos, FRAME_CONVENTION fc) const {
        double lat, lon, h;
        CartToGeo(cartPos.GetX(), cartPos.GetY(), cartPos.GetZ(),lat, lon, h,fc);
        return {lat,lon,h};
    }

    void FrGeographicServices::CartToGeo(const Position& cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const {
        geoCoord = CartToGeo(cartPos,fc);
    }


    double FrGeographicServices::GetDeclinationFromCart(const Position &cartPos, double year, FRAME_CONVENTION fc) const {
        /// Convert the node local coordinates to geographical coordinates
        auto geoCoord = CartToGeo(cartPos, fc);

        return GetDeclinationFromGeo(geoCoord, year);
    }

    double FrGeographicServices::GetDeclinationFromCart(double x, double y, double z,
                                                        double year, FRAME_CONVENTION fc) const {
        return GetDeclinationFromCart(Position(x,y,z), year, fc);
    }

    double
    FrGeographicServices::GetDeclinationFromGeo(const FrGeographicCoord &geoCoord, double year) const {

        // FIXME : le chemin vers le modèle magnétique est indiqué via la variable d'environnement GEOGRAPHICLIB_MAGNETIC_PATH
        // A voir si on doit changer ça.
//        putenv("GEOGRAPHICLIB_MAGNETIC_PATH=/home/d-ice/Documents/DEV/frydom_GPL/cmake-build-debug/_deps/magneticmodel-src");

        /// Magnetic model loaded from _deps directory
        GeographicLib::MagneticModel magneticModel("emm2017", "../_deps/magneticmodel-src");

        /// Compute the magnetic declination
        double Bx, By, Bz, H, F, D, I;
        magneticModel(year, geoCoord.GetLatitude(), geoCoord.GetLongitude(), geoCoord.GetHeight(), Bx, By, Bz);
        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);

        return D;
    }

    double
    FrGeographicServices::GetDeclinationFromGeo(double lat, double lon, double h, double year) const {
        return GetDeclinationFromGeo(FrGeographicCoord(lat,lon,h),year);
    }


}
