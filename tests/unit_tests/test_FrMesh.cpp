//
// Created by lletourn on 19/07/19.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;
using namespace geom;


TEST(FrMesh,Polygon) {

    FRAME_CONVENTION fc = NWU;

    mesh::FrMesh mesh;

    mesh.CreateBox(10,10,10);

//    mesh.Write("testPolygon.obj");

    Position origin(0.,0.,0.);
//    origin.setRandom();

    Direction normal(1.,0.,1.);
    normal.normalize();

    auto plane = std::make_shared<geom::FrPlane>(origin, normal, fc);

    auto clippingSurface = std::make_shared<mesh::FrClippingPlane>(plane);

    mesh::FrMeshClipper clipper;
    clipper.SetClippingSurface(clippingSurface);

    clipper.Apply(&mesh);

    mesh.Write("testPolygon.obj");

    // Test on polygon
    auto polygonSet = mesh.GetBoundaryPolygonSet();

    EXPECT_TRUE(!polygonSet.empty());

    auto polygon = polygonSet[0];

    auto planeTest = polygon.GetPlane();
    Direction testNormal = normal - planeTest.GetNormal(fc);
    EXPECT_NEAR(testNormal.norm(),0,1E-8);

    EXPECT_NEAR(planeTest.GetDistanceToPoint(origin,fc), 0, 1E-8);

    //

}