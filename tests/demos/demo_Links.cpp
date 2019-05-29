//
// Created by lletourn on 27/05/19.
//

#include "frydom/frydom.h"
//#include "gtest/gtest.h"

using namespace frydom;

int main() {

    /** This demo features constraints between bodies, nodes, points, axis and planes. While kinematic links are used to
     * model realistic links between bodies, constraints are more abstract and can be used at a conceptual level.
     * Seven different constraints are defined : DistanceBetweenPoints, DistanceToAxis, PointOnPlane, PointOnline,
     * PlaneOnPlane, Perpendicular and Parallel. They are based either on points, axis and planes, and can be defined
     * within FRyDoM using respectively the FrPoint, FrAxis, and FrPlane classes. Those are just plain abstractions of
     * their geometric counterparts, based on FrNode. Since FrNodes belong to bodies, the constraints are applied in fine
     * on the bodies. It is however easier to define these constraints using the aforementioned classes rather than the
     * FrBody and FrNode directly.
     * The seven constraints are illustrated below; for convenience, one of the two body is set fixed in the world
     * reference frame and the other set free.
    */

    FrOffshoreSystem system;
    system.GetEnvironment()->ShowFreeSurface(false);
    system.GetEnvironment()->ShowSeabed(false);
    system.SetName("demo_Links");
    
    FRAME_CONVENTION fc = NWU;

    // Bodies definition
    auto fixedBody = system.NewBody();
    fixedBody->SetFixedInWorld(true);
    fixedBody->SetName("Fixed");
    fixedBody->SetColor(GoldenRod);

    auto movingBody = system.NewBody();
    movingBody->SetName("Moving");
    movingBody->SetColor(CornflowerBlue);

    auto movingToSuppNode = movingBody->NewNode();
    movingToSuppNode->SetPositionInWorld(Position(6,0,0), fc);

    auto suppBody = system.NewBody();
    suppBody->AllowCollision(false);
    makeItBox(suppBody, 10, 5, 2, 100);
    suppBody->SetName("Supp");
    suppBody->SetColor(DarkSeaGreen);
    suppBody->SetPosition(Position(6,0,0), fc);

    auto suppNode = suppBody->NewNode();

    auto fixedLinkBetweenMovingAndSuppBodies = make_fixed_link(movingToSuppNode, suppNode, &system);


    enum demo_cases {Cylindrical, Revolute, Spherical, Prismatic };
    demo_cases featuredCase = Prismatic;

    switch (featuredCase) {
        case Cylindrical: {

            /**
             * The DistanceBetweenPoints constraint illustrated here features a point located a one corner of a box
             * rotating around the center of a fixed sphere, at a given distance. The distance corresponds to the radius
             * of the sphere, so that the point on the box stay on the sphere surface.
             */

            // Definition of the fixed body, node and point
            makeItCylinder(fixedBody, 1, 40, 100);
            FrRotation fixedRotation; fixedRotation.RotX_DEGREES(5,fc);
            fixedBody->RotateAroundCOG(fixedRotation, fc);
            fixedBody->AllowCollision(false);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->RotateAroundXInBody(90*DEG2RAD,fc); // transforms Y axis in Z axis
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            // Definition of the moving body, node and point
            makeItCylinder(movingBody, 2, 20, 100);
            movingBody->AllowCollision(false);

            auto movingNode = movingBody->NewNode();
            movingNode->RotateAroundXInBody(90*DEG2RAD,fc); // transforms Y axis in Z axis
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            // Definition of the cylindrical link
            auto constraint = make_cylindrical_link(fixedNode, movingNode, &system);

            break;
        }

        case Revolute: {

            /**
             * The DistanceToAxis constraint illustrated here features a point at the corner of a moving box rotating
             * around the axis of the fixed cylinder, at a given distance (no autoDistance). The radius of the fixed
             * cylinder is set at the imposed distance in order to visualize easily the constraint.
             */

            // Definition of the fixed body, node and point
            makeItCylinder(fixedBody, 1, 40, 100);
            FrRotation fixedRotation; fixedRotation.RotX_DEGREES(5,fc);
            fixedBody->RotateAroundCOG(fixedRotation, fc);
            fixedBody->AllowCollision(false);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->RotateAroundXInBody(90*DEG2RAD,fc); // transforms Y axis in Z axis
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            // Definition of the moving body, node and point
            makeItCylinder(movingBody, 2, 20, 100);
            movingBody->AllowCollision(false);

            auto movingNode = movingBody->NewNode();
            movingNode->RotateAroundXInBody(90*DEG2RAD,fc); // transforms Y axis in Z axis
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            // Definition of the revolute link
            auto constraint = make_revolute_link(fixedNode, movingNode, &system);

            break;
        }
        case Spherical: {

            /**
             * The PointOnPlane constraint illustrated here features a point located at the bottom of a sphere moving on a
             * fixed plane. Since the plane is defined at the center of the fixed box, a distance corresponding to half
             * the thickness of the fixed box is imposed. It is only for illustration purpose, since it would have been
             * easier to locate the origin of the fixed plane, on the fixed box surface and rather than its center.
             * The contact on the sphere prevents it to slide on the plane, while the constraint prevents it to roll.
             * A second sphere without constraint, only contact, is added for comparison.
             */

            // Definition of the fixed body, node and point
            makeItBox(fixedBody, 1, 1, 10, 100);
            fixedBody->SetPosition(Position(0.,0.,6), fc);
            fixedBody->AllowCollision(false);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->SetPositionInBody(Position(0.,0.,-6),fc);
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            // Definition of the moving body, node and point
            makeItSphere(movingBody, 1, 100);
            FrRotation movingRotation; movingRotation.RotX_DEGREES(45,fc); // just to see more rotations
            movingBody->RotateAroundCOG(movingRotation,fc);
            movingBody->AllowCollision(false);

            auto movingNode = movingBody->NewNode();
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            // Definition of the spherical link
            auto constraint = make_spherical_link(fixedNode, movingNode, &system);

            break;
        }
        case Prismatic: {

            /**
             * The PointOnLine constraint illustrated here features a point located at the center of a sphere moving on a
             * the axis on a fixed cylinder.
             */

            // Definition of the fixed body, node and point
            makeItBox(fixedBody, 1, 40, 1, 100);
            FrRotation fixedRotation; fixedRotation.RotX_DEGREES(5,fc);
            fixedBody->RotateAroundCOG(fixedRotation, fc);
            fixedBody->AllowCollision(false);

            auto fixedNode = fixedBody->NewNode();
            fixedNode->RotateAroundXInBody(90*DEG2RAD,fc); // transforms Y axis in Z axis
            fixedNode->ShowAsset(true);
            fixedNode->GetAsset()->SetSize(10);

            // Definition of the moving body, node and point
            makeItBox(movingBody, 2, 20, 2, 100);
            movingBody->AllowCollision(false);

            auto movingNode = movingBody->NewNode();
            movingNode->RotateAroundXInBody(90*DEG2RAD,fc); // transforms Y axis in Z axis
            movingNode->ShowAsset(true);
            movingNode->GetAsset()->SetSize(10);

            // Definition of the prismatic link
            auto constraint = make_prismatic_link(fixedNode, movingNode, &system);

            break;
        }
    }

    // DoFullAssembly helps the bodies to adjust their position according to the constraints applied on them. It requires
    // constraints to be initialized and may not work with external forces, cables, etc. applied on bodies.
    system.Initialize();
    system.GetChronoSystem()->DoFullAssembly();

    // Run the simulation (or visualize the assembly)
    system.SetTimeStep(0.005);
    system.RunInViewer(5, 20, false);
//    system.Visualize(50, false);

    return 0;
}