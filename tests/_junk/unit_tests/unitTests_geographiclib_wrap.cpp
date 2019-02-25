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

#include <frydom/environment/FrEnvironment.h>
#include "frydom/environment/GeographicServices/FrGeographicServices.h"
//#include "fmt/format.h"

using namespace frydom;
int unitTest_FrGeographicServices(bool verbose = false){
    auto myGeoLib = std::make_shared<FrGeographicServices>();
    myGeoLib->SetGeographicOrigin(1.45, -6, 0);

    double north = 100, east = 200, down = 0;
    double lat, lon, h;
    myGeoLib->CartToGeo(north, east, down, lat, lon, h);
    if (verbose) {
        std::cout << "north = " << north << ", east = " << east << ", down = " << down << std::endl;
        std::cout<<"lat = "<< lat <<", lon = "<< lon <<", h = "<< h <<std::endl;
    }
    double xf, yf, zf;
    myGeoLib->GeoToCart(lat, lon, h, xf, yf, zf);
    if (verbose) {
        std::cout << "north = " << xf << ", east = " << yf << ", down = " << zf << std::endl;
    }
//     Pour tester cette fonctionnalité, modifier le chemin du fichier de modèle magnétique
//     dans FrGeographicServices.GetDeclinationFromCart :
//     GeographicLib::MagneticModel magneticModel("emm2017", "../../_deps/magneticmodel-src");
//
//    auto B = myGeoLib->GetDeclinationFromCart(north,east,down,2018);
//    if (verbose) {
//        std::cout << "B = " << B << std::endl;
//    }

    return ((north-xf)>1e-6 || (east-yf)>1e-6 || (down-zf)>1e-6);
}

int unitTest_GeoLib_integration_in_FrEnvironment(bool verbose = false){

    auto myEnvironment = std::make_shared<FrEnvironment>();
    /// We need here to initialize the TimeZone, for the year in ComputeMagneticDeclination.
    myEnvironment->GetTimeZone()->Initialize();

    myEnvironment->SetGeographicOrigin(1.45, -6, 0);

    double north = 100, east = 200, down = 0;
    double lat, lon, h;
    myEnvironment->Convert_CartToGeo(north,east,down,lat,lon,h);

    if (verbose) {
        std::cout << "north = " << north << ", east = " << east << ", down = " << down << std::endl;
        std::cout << "lat = " << lat << ", lon = " << lon << ", h = " << h << std::endl;
    }

    double xf, yf, zf;
    myEnvironment->Convert_GeoToCart(lat,lon,h,xf,yf,zf);
    if (verbose) {
        std::cout << "north = " << xf << ", east = " << yf << ", down = " << zf << std::endl;
    }

//     Pour tester cette fonctionnalité, modifier le chemin du fichier de modèle magnétique
//     dans FrGeographicServices.GetDeclinationFromCart :
//     GeographicLib::MagneticModel magneticModel("emm2017", "../../_deps/magneticmodel-src");
//    if (verbose) {
//        std::cout << "Year : " << myEnvironment->GetYear() << std::endl;
//    }
//
//    auto B = myEnvironment->GetDeclinationFromCart(north,east,down);
//    if (verbose) {
//        std::cout << "B = " << B << std::endl;
//    }

    return ((north-xf)>1e-6 || (east-yf)>1e-6 || (down-zf)>1e-6);

}

int main() {
    bool verbose = false;
    return(unitTest_FrGeographicServices(verbose) + unitTest_GeoLib_integration_in_FrEnvironment(verbose));

}
