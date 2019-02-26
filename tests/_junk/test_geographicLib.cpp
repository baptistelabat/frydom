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

#include "GeographicLib/LocalCartesian.hpp"
#include "fmt/format.h"


int main() {

    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                    GeographicLib::Constants::WGS84_f());

    // Reference
    const double lat0 = 47.253225, lon0 = -1.545559; // House


    double lat = 47.250719, lon = -1.546521; // D-ICE


    GeographicLib::LocalCartesian proj(lat0, lon0, 0., earth);

//    auto h0 = proj.HeightOrigin();
//    auto la0 = proj.LatitudeOrigin();
//    auto lo0 = proj.LongitudeOrigin();

    double x, y, z;
    proj.Forward(lat, lon, 0., x, y, z);

    fmt::print("Distances from house to D-ICE in meters : x={}; y={}; z={}\n\n", x, y, z);

//    mathutils::Vector3d<double> position(x, y, z);
//    std::cout << position << "\n\n";

//    fmt::print("Distance from house to D-ICE : {}\n\n", position.norm());
    fmt::print("Distance from house to D-ICE : {}\n\n", sqrt(x*x+y*y+z*z));


    double lat_rev, long_rev, h_rev;
    proj.Reverse(x, y, z, lat_rev, long_rev, h_rev);

    fmt::print("Reverse calculation\nLat={}\nLon={}\nh={}\n\n", lat_rev, long_rev, h_rev);



    return 0;
}