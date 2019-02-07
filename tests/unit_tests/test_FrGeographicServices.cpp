//
// Created by Lucas Letournel on 19/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrGeographicServices,GeographicServices){
    FRAME_CONVENTION fc = NWU;

    FrGeographicServices geoServices;

    FrGeographicCoord Origin(47.22, -1.55, 0.);
    geoServices.SetGeographicOrigin(Origin);

    // test GetGeographicOrigin
    FrGeographicCoord testOrigin = geoServices.GetGeographicOrigin();
    EXPECT_FLOAT_EQ(Origin.GetLatitude(), testOrigin.GetLatitude());
    EXPECT_FLOAT_EQ(Origin.GetLongitude(), testOrigin.GetLongitude());
    EXPECT_FLOAT_EQ(Origin.GetHeight(), testOrigin.GetHeight());

    double lat0, lon0, h0;
    geoServices.GetGeographicOrigin(lat0,lon0,h0);
    EXPECT_FLOAT_EQ(Origin.GetLatitude(), lat0);
    EXPECT_FLOAT_EQ(Origin.GetLongitude(), lon0);
    EXPECT_FLOAT_EQ(Origin.GetHeight(), h0);

    // test CartToGeo and GeoToCart
    Position cartPos(100., 96., 2.);
    FrGeographicCoord geoCoord;
    geoServices.CartToGeo(cartPos, geoCoord, fc);

    Position returnPos;
    geoServices.GeoToCart(geoCoord, returnPos, fc);

    Position testPosition = returnPos - cartPos;
//    EXPECT_TRUE(testPosition.isZero());
//    if (not(testPosition.isZero())){
//        std::cout<<testPosition<<std::endl;
//        std::cout<<cartPos<<std::endl;
//        std::cout<<returnPos<<std::endl;
//    }

    EXPECT_FLOAT_EQ(cartPos.GetX(), returnPos.GetX());
    EXPECT_FLOAT_EQ(cartPos.GetY(), returnPos.GetY());
    EXPECT_FLOAT_EQ(cartPos.GetZ(), returnPos.GetZ());

//     Pour tester cette fonctionnalité, modifier le chemin du fichier de modèle magnétique
//     dans FrGeographicServices.GetDeclinationFromCart :
//     GeographicLib::MagneticModel magneticModel("emm2017", "../../_deps/magneticmodel-src");
//    auto B = geoServices.GetDeclinationFromCart(cartPos, 2018, fc);
//    EXPECT_NEAR(-0.221/0.6, B, 1e-3);
//    std::cout << "decl = " << -0.221/0.6 << std::endl;
//    std::cout << "B = " << B << std::endl;

}

TEST(FrGeographicServices,FrBodyIntegration){
    FRAME_CONVENTION fc = NWU;
    FrOffshoreSystem_ system;

    FrGeographicCoord Origin(47.22, -1.55, 0.);

    auto GeoServices = system.GetEnvironment()->GetGeographicServices();
    GeoServices->SetGeographicOrigin(Origin);


    auto body = system.NewBody();

    Position bodyPosition(50.,92.,1.5);
    body->SetPosition(bodyPosition, fc);

    auto bodyGeoPos = body->GetGeoPosition(fc);
    auto geoPos = GeoServices->CartToGeo(bodyPosition, fc);
    EXPECT_FLOAT_EQ(bodyGeoPos.GetLatitude(),geoPos.GetLatitude());
    EXPECT_FLOAT_EQ(bodyGeoPos.GetLongitude(),geoPos.GetLongitude());
    EXPECT_FLOAT_EQ(bodyGeoPos.GetHeight(),geoPos.GetHeight());

    auto newGeoPos = GeoServices->CartToGeo(-bodyPosition, fc);
    body->SetGeoPosition(newGeoPos);

//    Position testPosition = body->GetPosition(fc);
//    EXPECT_TRUE(testPosition.isZero());
//    if (not(testPosition.isZero())){
//        std::cout<<testPosition<<std::endl;
//    }
    EXPECT_FLOAT_EQ(-bodyPosition.GetX(), body->GetPosition(fc).GetX());
    EXPECT_FLOAT_EQ(-bodyPosition.GetY(), body->GetPosition(fc).GetY());
    EXPECT_FLOAT_EQ(-bodyPosition.GetZ(), body->GetPosition(fc).GetZ());

}