//
// Created by lletourn on 27/05/19.
//

#include "frydom/frydom.h"
//#include "gtest/gtest.h"

using namespace frydom;

int main() {

    FrOffshoreSystem system;
    system.GetEnvironment()->ShowFreeSurface(false);
    system.GetEnvironment()->ShowSeabed(false);
    system.SetName("demo_Constraints");

    system.GetWorldBody()->AddSphereShape(10.);

    // Bodies definition
    auto fixedBody = system.NewBody();
    fixedBody->SetFixedInWorld(true);
    fixedBody->SetName("Fixed");
    fixedBody->SetColor(GoldenRod);

    auto movingBody = system.NewBody();
    movingBody->SetName("Moving");
    movingBody->SetColor(CornflowerBlue);

    enum demo_cases {DistanceBetweenPoints, DistanceToAxis };
    demo_cases featuredCase = DistanceToAxis;

    switch (featuredCase) {
        case DistanceBetweenPoints: {

            // Definition the fixed body, node and point
            makeItSphere(fixedBody, 10, 1000);
            fixedBody->AllowCollision(false);

            auto fixedNode = system.GetWorldBody()->NewNode();
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            auto fixedPoint = std::make_shared<FrPoint>(fixedNode);

            // Definition of the moving body, node and point
            makeItCylinder(movingBody, 2, 20, 1000);
            movingBody->AllowCollision(false);

            auto movingNode = movingBody->NewNode();
            movingNode->TranslateInBody(0, 10, 0, NWU);
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            auto movingPoint = std::make_shared<FrPoint>(movingNode);

            // Definition of the constraint between the two points
            double distance = 10.;
            auto constraint = make_constraint_distance_between_points(fixedPoint, movingPoint, &system, distance);

            break;
        }

        case DistanceToAxis: {

            // Definition the fixed body, node and point
            makeItCylinder(fixedBody, 1, 20, 1000);
            fixedBody->AllowCollision(false);

            auto fixedNode = system.GetWorldBody()->NewNode();
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            auto fixedAxis = std::make_shared<FrAxis>(fixedNode,XAXIS);

            // Definition of the moving body, node and point
            makeItSphere(movingBody, 10, 1000);
            movingBody->AllowCollision(false);

            auto movingNode = movingBody->NewNode();
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            auto movingPoint = std::make_shared<FrPoint>(movingNode);

            // Definition of the constraint of a distance of a point to an axis
            double distance = 10.;
            auto constraint = make_constraint_distance_to_axis(fixedAxis, movingPoint, &system, distance);

            break;
        }
    }

//    // Body1 definition
//    auto body1 = system.NewBody();
//    body1->SetName("1");
//    makeItBox(body1, 20, 10, 2, 1000);
//    body1->AllowCollision(false);
//    body1->SetColor(MediumVioletRed);
//
//    body1->SetPosition(Position(5.,5.,5.),NWU);
//
//    // Revolute link between body1 and world
//    auto node1 = body1->NewNode();
//    node1->TranslateInBody(10, 5, 0, NWU);
//    node1->ShowAsset(true);
//    node1->GetAsset()->SetSize(10);
////    node1->RotateAroundXInBody(90*DEG2RAD, NWU);
////    node1->RotateAroundZInBody(90*DEG2RAD, NWU);
//
//
//    auto nodeWorld = system.GetWorldBody()->NewNode();
//    nodeWorld->ShowAsset(true);
//    nodeWorld->GetAsset()->SetSize(10);
////    nodeWorld->TranslateInWorld(10, 0, 0, NWU);
////    nodeWorld->RotateAroundXInBody(45*DEG2RAD, NWU);
////    nodeWorld->RotateAroundZInBody(90*DEG2RAD, NWU);
//
//    auto point1 = std::make_shared<FrPoint>(node1);
//    auto pointWorld = std::make_shared<FrPoint>(nodeWorld);
//
//    auto axis1 = std::make_shared<FrAxis>(node1,YAXIS);
//    auto axisWorld = std::make_shared<FrAxis>(nodeWorld,XAXIS);
//
//    auto plane1 = std::make_shared<FrPlane>(node1,YAXIS);
//    auto planeWorld = std::make_shared<FrPlane>(nodeWorld,ZAXIS);
//
//    auto test = std::make_shared<FrConstraintDistanceBetweenPoints>(point1, pointWorld, &system, 10.);
//    //test->SetFlipped(true);
////    test->SetDistance(0.);
//    system.AddLink(test);

    system.Initialize();
    system.GetChronoSystem()->DoFullAssembly();

    // Run the simulation (or visualize the assembly)
    system.SetTimeStep(0.01);
    system.RunInViewer(0, 50, false);
//    system.Visualize(50, false);

    return 0;
}