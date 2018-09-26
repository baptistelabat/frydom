//
// Created by Lucas Letournel on 26/09/18.
//

#include <frydom/environment/FrEnvironment.h>
#include "frydom/utils/FrGeographicServices.h"
//#include "fmt/format.h"

using namespace frydom;
int unitTest_FrGeographicServices(bool verbose = false){
    auto myGeoLib = std::make_shared<FrGeographicServices>();
    myGeoLib->SetGeographicOrigin(1.45, -6, 0);

    double north = 100, east = 200, down = 0;
    double lat, lon, h;
    myGeoLib->Convert_CartToGeo(north,east,down,lat,lon,h);
    if (verbose) {
        std::cout << "north = " << north << ", east = " << east << ", down = " << down << std::endl;
        std::cout<<"lat = "<< lat <<", lon = "<< lon <<", h = "<< h <<std::endl;
    }
    double xf, yf, zf;
    myGeoLib->Convert_GeoToCart(lat,lon,h,xf,yf,zf);
    if (verbose) {
        std::cout << "north = " << xf << ", east = " << yf << ", down = " << zf << std::endl;
    }
//     Pour tester cette fonctionnalité, modifier le chemin du fichier de modèle magnétique
//     dans FrGeographicServices.ComputeMagneticDeclination :
//     GeographicLib::MagneticModel magneticModel("emm2017", "../../_deps/magneticmodel-src");
//
//    auto B = myGeoLib->ComputeMagneticDeclination(north,east,down,2018);
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
//     dans FrGeographicServices.ComputeMagneticDeclination :
//     GeographicLib::MagneticModel magneticModel("emm2017", "../../_deps/magneticmodel-src");
//    if (verbose) {
//        std::cout << "Year : " << myEnvironment->GetYear() << std::endl;
//    }
//
//    auto B = myEnvironment->ComputeMagneticDeclination(north,east,down);
//    if (verbose) {
//        std::cout << "B = " << B << std::endl;
//    }

    return ((north-xf)>1e-6 || (east-yf)>1e-6 || (down-zf)>1e-6);

}

int main() {
    bool verbose = false;
    return(unitTest_FrGeographicServices(verbose) + unitTest_GeoLib_integration_in_FrEnvironment(verbose));

}

