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

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrGeographicServices, GeographicServices) {
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
  geoServices.GetGeographicOrigin(lat0, lon0, h0);
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

TEST(FrGeographicServices, FrBodyIntegration) {
  FRAME_CONVENTION fc = NWU;
  FrOffshoreSystem system("test_GeographicService");

  FrGeographicCoord Origin(47.22, -1.55, 0.);

  auto GeoServices = system.GetEnvironment()->GetGeographicServices();
  GeoServices->SetGeographicOrigin(Origin);


  auto body = system.NewBody("body");

  Position bodyPosition(50., 92., 1.5);
  body->SetPosition(bodyPosition, fc);

  auto bodyGeoPos = body->GetGeoPosition(fc);
  auto geoPos = GeoServices->CartToGeo(bodyPosition, fc);
  EXPECT_FLOAT_EQ(bodyGeoPos.GetLatitude(), geoPos.GetLatitude());
  EXPECT_FLOAT_EQ(bodyGeoPos.GetLongitude(), geoPos.GetLongitude());
  EXPECT_FLOAT_EQ(bodyGeoPos.GetHeight(), geoPos.GetHeight());

  auto newGeoPos = GeoServices->CartToGeo(-bodyPosition, fc);
  body->SetGeoPosition(newGeoPos);

  EXPECT_FLOAT_EQ(-bodyPosition.GetX(), body->GetPosition(fc).GetX());
  EXPECT_FLOAT_EQ(-bodyPosition.GetY(), body->GetPosition(fc).GetY());
  EXPECT_FLOAT_EQ(-bodyPosition.GetZ(), body->GetPosition(fc).GetZ());

}
