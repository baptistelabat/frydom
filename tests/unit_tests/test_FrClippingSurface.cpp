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

TEST(FrClippingPlaneTest, GetDistance) {

  FRAME_CONVENTION fc = NWU;

  double x = 1., y = 5., z = 8.;

  auto planeXOY = std::make_shared<geom::FrPlane>(Position(), Direction(0., 0., 1.), fc);
  mesh::FrClippingPlane clippingPlaneXOY(planeXOY);

  auto planeYOZ = std::make_shared<geom::FrPlane>(Position(), Direction(1., 0., 0.), fc);
  mesh::FrClippingPlane clippingPlaneYOZ(planeYOZ);

  auto planeZOX = std::make_shared<geom::FrPlane>(Position(), Direction(0., 1, 0.), fc);
  mesh::FrClippingPlane clippingPlaneZOX(planeZOX);

  EXPECT_NEAR(clippingPlaneXOY.GetDistance(mesh::FrMesh::Point(x, y, z)), z, 1E-16);
  EXPECT_NEAR(clippingPlaneYOZ.GetDistance(mesh::FrMesh::Point(x, y, z)), x, 1E-16);
  EXPECT_NEAR(clippingPlaneZOX.GetDistance(mesh::FrMesh::Point(x, y, z)), y, 1E-16);

}

TEST(FrClippingPlaneTest, GetIntersection) {

  FRAME_CONVENTION fc = NWU;

  VectorT<double, 3> p0(1., 5., 8.);
  VectorT<double, 3> p1(1., 5., 3.);

  auto planeXOY = std::make_shared<geom::FrPlane>(Position(1, 2, -3), Direction(0., 0., 1.), fc);
  mesh::FrClippingPlane clippingPlaneXOY(planeXOY);
  clippingPlaneXOY.SetBodyPosition(Position(6, 5, 4));

//    std::cout<<clippingPlaneXOY.GetIntersection(p0, p1)<<std::endl;



  VectorT<double, 3> p2(1., 5., 8.);
  VectorT<double, 3> p3(1., 8., 8.);

  auto planeXOZ = std::make_shared<geom::FrPlane>(Position(1, 2, -3), Direction(0., 1., 0.), fc);
  mesh::FrClippingPlane clippingPlaneXOZ(planeXOZ);
  clippingPlaneXOZ.SetBodyPosition(Position(6, 5, 4));

//    std::cout<<clippingPlaneXOZ.GetIntersection(p2, p3)<<std::endl;

}