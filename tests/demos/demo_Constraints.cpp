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

    // Bodies definition
    auto fixedBody = system.NewBody();
    fixedBody->SetFixedInWorld(true);
    fixedBody->SetName("Fixed");
    fixedBody->SetColor(GoldenRod);

    auto movingBody = system.NewBody();
    movingBody->SetName("Moving");
    movingBody->SetColor(CornflowerBlue);

    enum demo_cases {DistanceBetweenPoints, DistanceToAxis, PointOnPlane, PlaneOnPlane, Perpendicular };
    demo_cases featuredCase = Perpendicular;

    switch (featuredCase) {
        case DistanceBetweenPoints: {

            // Definition the fixed body, node and point
            makeItSphere(fixedBody, 10, 1000);
            fixedBody->AllowCollision(false);

            auto fixedNode = fixedBody->NewNode();
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
            makeItCylinder(fixedBody, 10, 20, 100);
            fixedBody->AllowCollision(false);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            auto fixedAxis = std::make_shared<FrAxis>(fixedNode,YAXIS);

            // Definition of the moving body, node and point
            makeItCylinder(movingBody, 1, 20, 100);
            movingBody->AllowCollision(false);
            movingBody->SetPosition(Position(10,0,0), NWU);

            auto movingNode = movingBody->NewNode();
            movingNode->TranslateInBody(0,10,0, NWU);
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            auto movingPoint = std::make_shared<FrPoint>(movingNode);

            // Definition of the constraint of a distance of a point to an axis
            double distance = 10.;
            auto constraint = make_constraint_distance_to_axis(fixedAxis, movingPoint, &system, false, distance);

            break;
        }
        case PointOnPlane: {

            // Definition the fixed body, node and point
            makeItBox(fixedBody, 30, 30, 1, 100);
            fixedBody->AllowCollision(false);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            auto fixedPlane = std::make_shared<FrPlane>(fixedNode,ZAXIS);

            // Definition of the moving body, node and point
            makeItBox(movingBody, 10, 5, 1, 100);
            movingBody->AllowCollision(false);
//            movingBody->SetPosition(Position(10,0,0), NWU);

            auto movingNode = movingBody->NewNode();
            movingNode->TranslateInBody(5,2.5,0.5, NWU);
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            auto movingPoint = std::make_shared<FrPoint>(movingNode);

            // Definition of the point on plane constraint
            double distance = -0.5;
            auto constraint = make_constraint_point_on_plane(fixedPlane, movingPoint, &system, distance);

            break;
        }
        case PlaneOnPlane: {

            // Definition the fixed body, node and point
            makeItBox(fixedBody, 30, 30, 1, 100);
            fixedBody->AllowCollision(false);
            FrRotation fixedRotation; fixedRotation.RotX_DEGREES(15,NWU);
            fixedBody->RotateAroundCOG(fixedRotation, NWU);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            auto fixedPlane = std::make_shared<FrPlane>(fixedNode,ZAXIS);

            // Definition of the moving body, node and point
            makeItBox(movingBody, 10, 5, 1, 100);
            movingBody->AllowCollision(false);
//            movingBody->SetPosition(Position(10,0,0), NWU);

            auto movingNode = movingBody->NewNode();
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            auto movingPlane = std::make_shared<FrPlane>(movingNode,ZAXIS);

            // Definition of the plane on plane constraint
            bool flipped = false;
            double distance = -1;
            auto constraint = make_constraint_plane_on_plane(fixedPlane, movingPlane, &system, flipped, distance);

            break;
        }
        case Perpendicular:{

            // Definition the fixed body, node and point
            makeItCylinder(fixedBody, 1, 10, 100);
            fixedBody->AllowCollision(false);
            FrRotation fixedRotation; fixedRotation.RotX_DEGREES(15,NWU);
            fixedBody->RotateAroundCOG(fixedRotation, NWU);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            auto fixedAxis = std::make_shared<FrAxis>(fixedNode,YAXIS);

            // Definition of the moving body, node and point
            makeItCylinder(movingBody, 1, 10, 100);
            movingBody->AllowCollision(false);

            auto movingNode = movingBody->NewNode();
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            auto movingAxis = std::make_shared<FrAxis>(movingNode,YAXIS);

            // Definition of the perpendicular constraint
            auto constraint = make_constraint_perpendicular(fixedAxis, movingAxis, &system);

            break;
        }
    }

    system.Initialize();
    system.GetChronoSystem()->DoFullAssembly();

    // Run the simulation (or visualize the assembly)
    system.SetTimeStep(0.005);
    system.RunInViewer(0, 50, false);
//    system.Visualize(50, false);

    return 0;
}