//
// Created by Lucas Letournel on 26/09/18.
//

#include "frydom/utils/FrGeographicServices.h"
//#include "fmt/format.h"

using namespace frydom;
int main() {

    auto myGeoLib = std::make_shared<FrGeographicServices>();

    double north = 100, east = 200, down = 0;
    double lat, lon, h;
    myGeoLib->Convert_CartToGeo(north,east,down,lat,lon,h);

    std::cout<<"north = "<< north <<", east = "<< east <<", down = "<< down <<std::endl;
    std::cout<<"lat = "<< lat <<", lon = "<< lon <<", h = "<< h <<std::endl;

    double xf, yf, zf;
    myGeoLib->Convert_GeoToCart(lat,lon,h,xf,yf,zf);
    std::cout<<"north = "<< xf <<", east = "<< yf <<", down = "<< zf <<std::endl;

    return ((north-xf)>1e-6 || (east-yf)>1e-6 || (down-zf)>1e-6);

    // Pour tester cette fonctionnalité, modifier le chemin du fichier de modèle magnétique
    // dans FrGeographicServices.ComputeMagneticDeclination :
    // GeographicLib::MagneticModel magneticModel("emm2017", "../../_deps/magneticmodel-src");

//    auto B = myGeoLib->ComputeMagneticDeclination(north,east,down,2018,myFrame);
//    std::cout<<"B = "<<B;

}

