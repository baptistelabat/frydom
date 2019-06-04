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

    /** We present in this demo the methods which can be used to defined an body. They can be grouped in different thematics:
     *      - the methods to set the visual assets of the body, in terms of shapes and colors
     *      - the methods to set the inertia parameters of the body (mass, inertia tensor, center of gravity)
     *      - the methods to add nodes to the body, which can be used for setting kinematic links, cables, etc. between bodies
     *      - the methods to set the position and rotation of the body, or to translate and rotate the body.
     *
     */

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem system;

    // Specify the size of the free surface asset.
    auto FSAsset = system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset();
    FSAsset->SetGrid(-100., 100, 2, -100, 100, 2);


    // Creating a new body is really simple, you just have to call the NewBody() method of offshore system. This way,
    // it is automatically linked to the system, with the right contact method defined in the offshore system.
    auto body = system.NewBody();

    // Give a name to your new body.
    body->SetName("Buddy");

    // Make the body fixed : If true, it does not move respect to the absolute world, despite constraints, forces, etc.
    // By default a body is not fixed.
    body->SetFixedInWorld(true);

    // You can also constraint the body motion to some degree of freedom, a particular plane, etc.
    // The body is then linked to the world, using a DOF mask.
    body->GetDOFMask()->MakeItLocked();

    // =============================================================================================================
    // VISUAL AND COLLISION ASSETS
    // =============================================================================================================

    // Add a visual asset to your body. It is only a matter of visualization, you can have a visual asset which is
    // completely different from the one you used to compute hydrodynamic loads.

    double radius = 5, height = 10., xSize = 5., ySize = 5., zSize = 10., Mass = 1000.;
    // Three default visual assets are available : sphere, cylinder and box shaped assets.
    body->AddSphereShape(radius); // radius = 20m
//    body->AddCylinderShape(radius, height); // radius = 10m, height = 20m
//    body->AddBoxShape(xSize, ySize, zSize); // xSize = 5m, ySize = 10m, zSize = 20m

    // Add a collision asset to your body, it allows the body to collide with other elements. You can add collision mesh
    // based on triangular mesh as well.
    auto sphereCollision = std::make_shared<FrCollisionModel>();
    sphereCollision->AddSphere(radius,Position());
    body->SetCollisionModel(sphereCollision);

    // However you can make your body a sphere, cylinder or box using the following functions :
    // These functions set the inertia parameters, the collision model and the visual asset.
//    makeItSphere(body, radius, Mass);
//    makeItCylinder(body,radius, height, Mass);
//    makeItBox(body,xSize, ySize, zSize, Mass);

    // For adding your own mesh, specify a WaveFront .obj file name. Meshmagick software can help you convert other
    // mesh format into .obj : https://github.com/LHEEA/meshmagick.
//    body->AddMeshAsset("ship.obj");

    // Select the color of your body, in the NAMED_COLOR enum (FrColors.h for more details).
    body->SetColor(IndianRed);

    // =============================================================================================================
    // PRINCIPAL INERTIAL PARAMETERS
    // =============================================================================================================

    // The inertia parameters, mass, inertia matrix and application point, are strongly linked, so we embedded them into
    // a single element: the inertia tensor (FrInertiaTensor). We chose to fix the application point to the body CoG by
    // convention. So, for setting mass and CoG position for a body, you just need to use the SetInertiaTensor() method.

    // Note that the simulation won't work with null  diagonal inertia parameters and non fixed body.
    double mass = 1000.; // in Kg
    double Ixx =1., Iyy = 1., Izz = 1., Ixy = 0., Ixz = 0., Iyz = 0.;

    // You can set the inertia coefficients, given at the COG frame (position and orientation, which may
    // not be the body orientation).

    Position COGPos(10.,-5.,-3.);
    FrRotation COGOrientation;
    COGOrientation.SetCardanAngles_DEGREES(0.,0.,90.,fc); // COGFrame has a 90 yaw rotation, compared to the body reference frame.

    FrFrame COGFrame(COGPos,COGOrientation,fc); // creating COGFrame, from the COG position and orientation.
    FrInertiaTensor InertiaTensor(mass,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,COGFrame,fc);

    // Or you may got your inertia coefficients expressed in a frame, auxFrame, that can be different from the
    // COGPosition. The inertia are then automatically transported to the CoG Position specified, by this following
    // constructor.
    FrFrame auxFrame; // defined from the position and orientation you got.

    FrInertiaTensor InertiaTensor0(mass,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,auxFrame,COGPos,fc);

    // Set now the inertia tensor you want in the body.
    body->SetInertiaTensor(InertiaTensor);

    // =============================================================================================================
    // NODES
    // =============================================================================================================

    // Nodes are simple elements, belonging to at least one body, which are used to easily setup cable connections,
    // kinematic links between bodies, etc. As such, they contain a frame and follow their respective body. Their
    // position, velocity and acceleration are thus updated by their respective body.
    //
    // You can add nodes to a body with a simple method as well. You can specify the position of the node in the body
    // reference frame, its orientation, or directly the combination of both with a frame.

    // Position of the node, expressed in the body reference frame
    Position NodePos(1.,0.,5.);
    auto Node0 = body->NewNode();
    Node0->SetPositionInBody(NodePos,fc);

    // Frame of the node, expressed in the body reference frame
    FrRotation NodeRot;
    NodeRot.SetNullRotation();
    FrFrame frameNode1(NodePos, NodeRot, fc);
    auto Node1 = body->NewNode();
    Node1->SetFrameInBody(frameNode1);

    // =============================================================================================================
    // POSITIONS
    // =============================================================================================================

    // Different methods are available to set the body position, orientation or frame (position+rotation) in the world
    // reference frame. Be careful to take care of the frame convention (NED/NWU) when setting (and getting) positions.
    // Note that these methods moves the entire body, along with its nodes and other attached elements.

    //------------------------------------------------------------------------------------------------------------------
    // SetPosition and translation methods:

    // Set the body reference frame position at bodyPosInWorld, expressed in world reference frame.
    Position bodyPosInWorld(0.,1.,2.);
    body->SetPosition(bodyPosInWorld,fc);

    // Set the body reference frame at a geographic coordinate (Guinea Golf). Don't forget to define first the origin of
    // geographic coordinates, see demo_Environment.
    body->SetGeoPosition(FrGeographicCoord(0.,0.,0.));

    // The bodyPoint, expressed in the body reference frame, will be moved to the worldPos, expressed in the world
    // reference frame. All the body translate accordingly.
    Position bodyPoint(1.,0.,5.), worldPos(9.,0.,7.);
    body->SetPositionOfBodyPoint(bodyPoint, worldPos, fc);

    // Translate the body of a vector bodyTranslation, expressed in the body reference frame.
    Position bodyTranslation(10.,0.,0.);
    body->TranslateInBody(bodyTranslation, fc);

    // Translate the body of a vector worldTranslation, expressed in the world reference frame.
    Position worldTranslation(0.,50.,0.);
    body->TranslateInWorld(worldTranslation, fc);

    //------------------------------------------------------------------------------------------------------------------
    // SetRotation and Rotation methods : For constructing rotations, see demo_Frame_and_Rotation.

    // Set the body reference frame orientation, using quaternions
    FrUnitQuaternion IdQuaternion(1.,0.,0.,0.,fc); // Id quaternion means no rotation.
    body->SetRotation(IdQuaternion);
    // or rotation, based on quaternions:
    FrRotation bodyRotation(IdQuaternion);
    body->SetRotation(bodyRotation);

    // Rotate the body around its reference frame, based on its current orientation, using quaternions or a rotation.
    body->Rotate(IdQuaternion);
//    body->Rotate(bodyRotation);

    // Rotate the body around its COG, based on its current orientation, using quaternions or a rotation.
    body->RotateAroundCOG(IdQuaternion, fc);
//    body->RotateAroundCOG(bodyRotation, fc);

    // Rotate the body around a point position, expressed in world reference frame, based on its current orientation,
    // using quaternions or a rotation.
    body->RotateAroundPointInWorld(IdQuaternion, worldPos, fc);
//    body->RotateAroundPointInWorld(bodyRotation, worldPos, fc);

    // Rotate the body around a point position, expressed in body reference frame, based on its current orientation,
    // using quaternions or a rotation.
    body->RotateAroundPointInBody(IdQuaternion, bodyPoint, fc);
//    body->RotateAroundPointInBody(bodyRotation, bodyPoint, fc);


    // ------------------ Run ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.04);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 30s) and the distance from the camera to the objectif (100m).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(30, 100, false);

}
