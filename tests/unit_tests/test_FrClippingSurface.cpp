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

TEST(FrClippingPlaneTest,GetDistance) {

    double x = 1., y = 5., z = 8.;

    FrOffshoreSystem system;
    auto body = system.NewBody();
    auto node = body->NewNode();

    auto planeXOY = std::make_shared<FrCPlane>(node);
    mesh::FrClippingPlane clippingPlaneXOY(planeXOY);

    auto planeYOZ = std::make_shared<FrCPlane>(node, XAXIS);
    mesh::FrClippingPlane clippingPlaneYOZ(planeYOZ);

    auto planeZOX = std::make_shared<FrCPlane>(node, YAXIS);
    mesh::FrClippingPlane clippingPlaneZOX(planeZOX);

    EXPECT_NEAR(clippingPlaneXOY.GetDistance(mesh::FrMesh::Point(x, y , z)), z, 1E-16);
    EXPECT_NEAR(clippingPlaneYOZ.GetDistance(mesh::FrMesh::Point(x, y , z)), x, 1E-16);
    EXPECT_NEAR(clippingPlaneZOX.GetDistance(mesh::FrMesh::Point(x, y , z)), y, 1E-16);

}

TEST(FrClippingPlaneTest,GetIntersection) {

    VectorT<double, 3> p0 (1., 5. , 8.);
    VectorT<double, 3> p1 (1., 5. , 3.);

    FrOffshoreSystem system;
    auto body = system.NewBody();
    auto node = body->NewNode();
    node->SetPositionInBody(Position(1,2,-3), NWU);

    auto planeXOY = std::make_shared<FrCPlane>(node);
    mesh::FrClippingPlane clippingPlaneXOY(planeXOY);
    clippingPlaneXOY.SetBodyPosition(Position(6,5,4));

    std::cout<<clippingPlaneXOY.GetIntersection(p0, p1)<<std::endl;



    VectorT<double, 3> p2 (1., 5. , 8.);
    VectorT<double, 3> p3 (1., 8. , 8.);

    auto planeXOZ = std::make_shared<FrCPlane>(node, YAXIS);
    mesh::FrClippingPlane clippingPlaneXOZ(planeXOZ);
    clippingPlaneXOZ.SetBodyPosition(Position(6,5,4));

    std::cout<<clippingPlaneXOZ.GetIntersection(p2, p3)<<std::endl;

}