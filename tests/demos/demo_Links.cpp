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

using namespace frydom;

int main(int argc, char* argv[]) {

    /** This demo features kinematic links between bodies : Fixed, Revolute, Cylindrical, Prismatic and Spherical. These
     * are used to model realistic links; for more abstract constraints between bodies, see demo_Constraints (featuring
     * DistanceBetweenPoints, DistanceToAxis, PointOnPlane, PointOnline, PlaneOnPlane, Perpendicular and Parallel)
     *
     * Kinematic links are based on FrNodes, belonging to bodies, to locate and orientate the kinematic link frames.
     *
     * For convenience, one of the two body is set fixed in the world reference frame and the other set free.
    */

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;

    // Create the offshore system and disable the free surface and seabed visualizations
    FrOffshoreSystem system;
    system.GetEnvironment()->ShowFreeSurface(false);
    system.GetEnvironment()->ShowSeabed(false);
    system.SetName("demo_Links");
    system.GetPathManager()->SetResourcesPath(std::string(RESOURCES_PATH));

    // Bodies definition
    auto fixedBody = system.NewBody();
    fixedBody->SetFixedInWorld(true);
    fixedBody->SetName("Fixed");
    fixedBody->SetColor(GoldenRod);

    auto movingBody = system.NewBody();
    movingBody->SetName("Moving");
    movingBody->SetColor(CornflowerBlue);


    // A suppBody is added to the movingBody, using a FixedLink, to illustrate the different kinematic links below.
    auto suppBody = system.NewBody();
    suppBody->AllowCollision(false);
    makeItBox(suppBody, 10, 5, 2, 100);
    suppBody->SetName("Supp");
    suppBody->SetColor(DarkSeaGreen);
    suppBody->SetPosition(Position(6,0,0), fc);

    auto movingToSuppNode = movingBody->NewNode();
    movingToSuppNode->SetPositionInWorld(Position(6,0,0), fc);

    auto suppNode = suppBody->NewNode();

    auto fixedLinkBetweenMovingAndSuppBodies = make_fixed_link(movingToSuppNode, suppNode, &system);


    enum demo_cases {Cylindrical, Revolute, Spherical, Prismatic };
    demo_cases featuredCase = Cylindrical;

    switch (featuredCase) {
        case Cylindrical: {

            /**
             * The Cylindrical link illustrated here features a cylinder rotating and translating around the axe of
             * another cylinder. By definition, the Cylindrical link degrees of freedom (translation and rotation) are
             * around the Z axis.
             * The makeItCylinder instantiate a cylinder with an axis around its Y axis. We then need to transform the
             * node frame in order to get the Y axis to be the Z axis.
             * The fixedBody is rotated around X, so that the movingBody can slide, due to gravity.
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
             * The Revolute link illustrated here features a cylinder rotating around the axe of another cylinder.
             * By definition, the Revolute link rotation is around the Z axis.
             * The makeItCylinder instantiate a cylinder with an axis around its Y axis. We then need to transform the
             * node frame in order to get the Y axis to be the Z axis.
             * The fixedBody is rotated around X, but in contrary to the Cylindrical link, the movingBody can't slide
             * along the Z Axis with the Revolute link.
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
             * The movingBody, featured as a sphere, illustrates the Spherical link between the two boxes.
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
             * The Prismatic link illustrated here features a box sliding along the main direction of another box.
             * By definition, the Prismatic link translation is along the Z axis.
             *
             * The fixedBody main direction is taken as its Y axis and slightly rotated, so the movingBody slides slowly.
             * We then need to transform the node frame in order to get the Y axis to be the Z axis.
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
    system.DoAssembly();

    // Run the simulation (or visualize the assembly)
    system.SetTimeStep(0.01);
    system.RunInViewer(5, 20, false);
//    system.Visualize(50, false);

    return 0;
}