//
// Created by Lucas Letournel on 26/09/18.
//

#include "frydom/utils/FrGeographicServices.h"
//#include "fmt/format.h"

using namespace frydom;
int main() {

    auto myGeoLib = std::make_shared<FrGeographicServices>();

    FrFrame myFrame = NED;

    double x = 100, y = 200, z = 0;
    double lat, lon, h;
    myGeoLib->Convert_CartToGeo(x,y,z,lat,lon,h,myFrame);

    std::cout<<"x = "<< x <<", y = "<< y <<", z = "<< z <<std::endl;
    std::cout<<"lat = "<< lat <<", lon = "<< lon <<", h = "<< h <<std::endl;

    double xf, yf, zf;
    myGeoLib->Convert_GeoToCart(lat,lon,h,xf,yf,zf,myFrame);
    std::cout<<"x = "<< xf <<", y = "<< yf <<", z = "<< zf <<std::endl;

    return ((x-xf)>1e-6 || (y-yf)>1e-6 || (z-zf)>1e-6);

    // Pour tester cette fonctionnalité, modifier le chemin du fichier de modèle magnétique
    // dans FrGeographicServices.ComputeMagneticDeclination :
    // GeographicLib::MagneticModel magneticModel("emm2017", "../../_deps/magneticmodel-src");

//    auto B = myGeoLib->ComputeMagneticDeclination(x,y,z,2018,myFrame);
//    std::cout<<"B = "<<B;

}

