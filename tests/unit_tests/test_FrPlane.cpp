//
// Created by lletourn on 17/07/19.
//


#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;
using namespace geom;

TEST(FrPlane,Construction) {

    std::srand((unsigned int) time(nullptr));

    FRAME_CONVENTION fc = NWU;

    Position origin;
    origin.setRandom();

    Direction normal;
    normal.setRandom();
    normal.normalize();

    FrPlane plane(origin, normal, fc);

    Position testOrigin = origin - plane.GetOrigin(fc);
    EXPECT_NEAR(testOrigin.norm(), 0., 1E-8);

    Position testNormal = normal - plane.GetNormal(fc);
    EXPECT_NEAR(testNormal.norm(), 0., 1E-8);

    // Distance
    Position P0;
    P0.setRandom();

    double distance = normal.dot(P0 - origin);
    EXPECT_NEAR(distance, plane.GetSignedDistanceToPoint(P0,fc), 1E-8);

    distance = std::abs(distance);
    EXPECT_NEAR(distance, plane.GetDistanceToPoint(P0,fc), 1E-8);

    // Intersection with a line, normal to the plan
    {
        Position P1 = P0 + normal;

        // P0O
        Direction vector = origin - P0;
        // s_i
        double s = vector.dot(normal);
        Position intersection = P0 + normal * s;

        Position testIntersection = intersection - plane.GetClosestPointOnPlane(P0, fc);
        EXPECT_NEAR(testIntersection.norm(), 0., 1E-8);
    }

    // Intersection with an arbitrary line
    {
        Position P1;
        P1.setRandom();
        Direction line = (P1 - P0);
        // P0O
        Direction vector = origin - P0;
        // s_i
        double s = vector.dot(normal) / line.dot(normal);
        // P_i
        Position intersection = P0 + (P1 - P0) * s;

        Position testIntersection = intersection - plane.GetIntersectionWithLine(P0, P1, fc);
        EXPECT_NEAR(testIntersection.norm(), 0., 1E-8);
    }

    // Intersection with a line passing through the origin
    Position testIntersection = origin - plane.GetIntersectionWithLine(P0,origin,fc);
    EXPECT_NEAR(testIntersection.norm(), 0., 1E-8);


    // Frame
    {
        auto planeFrame = plane.GetFrame();

        auto P1 = plane.GetClosestPointOnPlane(P0, fc);

        auto P1inPlane = planeFrame.GetPointPositionInFrame(P1,NWU);
        EXPECT_NEAR(P1inPlane.GetZ(), 0., 1E-8);
    }

}

TEST(FrPlane,Cloud) {

    FRAME_CONVENTION fc = NWU;

    int nVertex = 4;
    double radius = 10;

    std::vector<Position> vertexList;

    Position origin(0.5,0,0.5);

    Direction normal(1,0,1);
    normal.normalize();

    FrPlane plane(origin, normal, fc);

    for (int i = 0; i<nVertex; i++) {
        Position P(radius*cos(2*MU_PI*i/nVertex), radius*sin(2*MU_PI*i/nVertex), 0);
        P = plane.GetFrame().GetPointPositionInParent(P, fc);
        vertexList.push_back(P);
    }

    FrPlane test(vertexList, fc);

    Direction testNormal = normal - test.GetNormal(fc);
    EXPECT_NEAR(testNormal.norm(),0,1E-8);

    EXPECT_NEAR(test.GetDistanceToPoint(Position(),fc)-plane.GetDistanceToPoint(Position(),fc),0,1E-8);

}