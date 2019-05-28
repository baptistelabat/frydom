//
// Created by lletourn on 27/05/19.
//

#include "frydom/frydom.h"
//#include "gtest/gtest.h"

using namespace frydom;

int main() {

    /** This demo features constraints between bodies, nodes, points, axis and planes. While kinematic links are used to
     * model realistic links between bodies, constraints are more abstract and can be used at a conceptual level.
     * Six different constraints are defined : DistanceBetweenPoints, DistanceToAxis, PointOnPlane, PlaneOnPlane,
     * Perpendicular and Parallel. They are based either on points, axis and planes, and can be defined within FRyDoM
     * using respectively the FrPoint, FrAxis, and FrPlane classes. Those are just plain abstractions of their geometric
     * counterparts, based on FrNode. Since FrNodes belong to bodies, the constraints are applied in fine on the bodies.
     * It is however easier to define these constraints using the aforementioned classes rather than the FrBody and
     * FrNode directly.
     * The six constraints are illustrated below; for convenience, one of the two body is set fixed in the world
     * reference frame and the other set free.
    */

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

    enum demo_cases {DistanceBetweenPoints, DistanceToAxis, PointOnPlane, PlaneOnPlane, Perpendicular, Parallel };
    demo_cases featuredCase = PointOnPlane;

    switch (featuredCase) {
        case DistanceBetweenPoints: {

            /**
             * The DistanceBetweenPoints constraint illustrated here features a point located a one corner of a box
             * rotating around the center of a fixed sphere, at a given distance. The distance corresponds to the radius
             * of the sphere, so that the point on the box stay on the sphere surface.
             */

            double distance = 10.;

            // Definition the fixed body, node and point
            makeItSphere(fixedBody, distance, 1000);
//            fixedBody->AllowCollision(false);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            auto fixedPoint = std::make_shared<FrPoint>(fixedNode);

            // Definition of the moving body, node and point
            makeItBox(movingBody, 10, 5, 1, 1000);
//            movingBody->AllowCollision(false);
            movingBody->SetPosition(Position(-20,0,0), NWU);

            auto movingNode = movingBody->NewNode();
            movingNode->TranslateInBody(5, 2.5, 0.5, NWU);
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            auto movingPoint = std::make_shared<FrPoint>(movingNode);

            // Definition of the constraint between the two points
            bool autoDistance = false;
            auto constraint = make_constraint_distance_between_points(fixedPoint, movingPoint, &system, autoDistance, distance);

            break;
        }

        case DistanceToAxis: {

            /**
             * The DistanceToAxis constraint illustrated here features a point at the corner of a moving box rotating
             * around the axis of the fixed cylinder, at a given distance (no autoDistance). The radius of the fixed
             * cylinder is set at the imposed distance in order to visualize easily the constraint.
             */

            double distance = 10.;

            // Definition the fixed body, node and point
            makeItCylinder(fixedBody, distance, 20, 100);
//            fixedBody->AllowCollision(false);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            auto fixedAxis = std::make_shared<FrAxis>(fixedNode,YAXIS);

            // Definition of the moving body, node and point
            makeItBox(movingBody, 10, 5, 1, 100);
//            movingBody->AllowCollision(false);
            movingBody->SetPosition(Position(-15,0,0), NWU);

            auto movingNode = movingBody->NewNode();
            movingNode->TranslateInBody(5, 2.5, 0.5, NWU);
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            auto movingPoint = std::make_shared<FrPoint>(movingNode);

            // Definition of the constraint of a distance of a point to an axis
            auto constraint = make_constraint_distance_to_axis(fixedAxis, movingPoint, &system, false, distance);

            break;
        }
        case PointOnPlane: {

            /**
             * The PointOnPlane constraint illustrated here features a point located at the bottom of a sphere moving on a
             * fixed plane. Since the plane is defined at the center of the fixed box, a distance corresponding to half
             * the thickness of the fixed box is imposed. It is only for illustration purpose, since it would have been
             * easier to locate the origin of the fixed plane, on the fixed box surface and rather than its center.
             * The contact on the sphere prevents it to slide on the plane, while the constraint prevents it to roll.
             * A second sphere without constraint, only contact, is added for comparison.
             */

            // Definition the fixed body, node and point
            makeItBox(fixedBody, 50, 50, 1, 100);
//            fixedBody->AllowCollision(false);
            FrRotation fixedRotation; fixedRotation.RotX_DEGREES(45,NWU);
            fixedBody->RotateAroundCOG(fixedRotation, NWU);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            auto fixedPlane = std::make_shared<FrPlane>(fixedNode,ZAXIS);

            // Definition of the moving body, node and point
            makeItSphere(movingBody, 5, 100);
//            movingBody->AllowCollision(false);
            movingBody->SetPosition(Position(-10,0,10), NWU);

            auto movingNode = movingBody->NewNode();
            movingNode->TranslateInBody(0,0,-5, NWU);
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            auto movingPoint = std::make_shared<FrPoint>(movingNode);

            // Definition of the point on plane constraint
            double distance = 0.5;
            auto constraint = make_constraint_point_on_plane(fixedPlane, movingPoint, &system, distance);

            auto freeBody = system.NewBody();
            makeItSphere(freeBody, 5, 100);
            freeBody->SetPosition(Position(10,0,7.5), NWU);
            freeBody->SetColor(DarkSeaGreen);

            break;
        }
        case PlaneOnPlane: {

            /**
             * The PlaneOnPlane constraint illustrated here, features a plane sliding on a fixed, slightly inclined one.
             * A distance between the plane is imposed since the planes are defined on the center of the boxes, and not
             * their surfaces. It is also here only for illustration purpose. Flipping the plane will flip the entire
             * box.
             */

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
            double distance = 1;
            auto constraint = make_constraint_plane_on_plane(fixedPlane, movingPlane, &system, flipped, distance);

            break;
        }
        case Perpendicular:{

            /**
             * the Perpendicular constraint illustrated here, features two axes: the first is fixed and represented by
             * the axis of the fixed cylinder. The second is represented by the axis of the falling cylinder.
             */

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
        case Parallel: {

            /**
             * the Parallel constraint illustrated here, features two axes: the first is fixed and represented by
             * the axis of the fixed cylinder. The second is represented by the axis of the falling cylinder.
             */

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
            auto constraint = make_constraint_parallel(fixedAxis, movingAxis, &system);

            break;
        }
    }

    // DoFullAssembly helps the bodies to adjust their position according to the constraints applied on them. It requires
    // constraints to be initialized and may not work with external forces, cables, etc. applied on bodies.
    system.Initialize();
    system.GetChronoSystem()->DoFullAssembly();

    // Run the simulation (or visualize the assembly)
    system.SetTimeStep(0.005);
    system.RunInViewer(0, 20, true);
//    system.Visualize(50, false);

    return 0;
}