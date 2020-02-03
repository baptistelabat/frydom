//
// Created by lletourn on 18/07/19.
//


#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;
using namespace geom;

TEST(FrPolygon, Construction) {

//    std::srand((unsigned int) time(nullptr));

  FRAME_CONVENTION fc = NWU;

  int nVertex = 100;
  double radius = 10;

  std::vector<Position> vertexList;

  Position origin(0.5, 0, 0.5);
//    origin.setRandom();

  Direction normal(1, 0, 1);
//    normal.setRandom();
  normal.normalize();

  FrPlane plane(origin, normal, fc);

  for (int i = 0; i < nVertex + 1; i++) {
    Position P(radius * cos(2 * MU_PI * i / nVertex), radius * sin(2 * MU_PI * i / nVertex), 0);
    P = plane.GetFrame().GetPointPositionInParent(P, fc);
    vertexList.push_back(P);
  }

  mesh::FrPolygon polygon(vertexList, fc);

  // Test plane given by polygon
  EXPECT_TRUE(polygon.IsPlanar());

  auto planeTest = polygon.GetPlane();
  Direction testNormal = normal - planeTest.GetNormal(fc);
  EXPECT_NEAR(testNormal.norm(), 0, 1E-8);

  EXPECT_NEAR(planeTest.GetDistanceToPoint(origin, fc), 0, 1E-8);

  // Tests integrals
  double area = MU_PI * radius * radius;
  EXPECT_TRUE((polygon.GetArea() - area) / area < 0.001);

  auto Ix = polygon.GetSurfaceIntegral(mesh::POLY_X);
  auto Iy = polygon.GetSurfaceIntegral(mesh::POLY_Y);
  auto Ixy = polygon.GetSurfaceIntegral(mesh::POLY_XY);
  auto Ix2 = polygon.GetSurfaceIntegral(mesh::POLY_X2);
  auto Iy2 = polygon.GetSurfaceIntegral(mesh::POLY_Y2);

  area = radius * radius * radius * radius * MU_PI / 4.;
  EXPECT_NEAR(Ix, 0, 1E-8);
  EXPECT_NEAR(Iy, 0, 1E-8);
  EXPECT_NEAR(Ixy, 0, 1E-8);
  EXPECT_TRUE((Ix2 - area) / area < 0.001);
  EXPECT_TRUE((Iy2 - area) / area < 0.001);

//    std::cout<<"Ix = "<< Ix<<", Iy = "<< Iy<<", Ixy = "<< Ixy<<", Ix2 = "<< Ix2<<", Iy2 = "<< Iy2<<std::endl;

}