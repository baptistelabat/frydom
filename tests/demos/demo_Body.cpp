//
// Created by Lucas Letournel on 20/12/18.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem_ system;

    // Creating a new body is really simple, you just have to call the NewBody() method of offshore system. This way,
    // it is automatically linked to the system, with the right contact method defined in the offshore system.
    auto body = system.NewBody();

    // Give a name to your new body.
    body->SetName("Buddy");

    // Make the body fixed : If true, it does not move respect to the absolute world, despite constraints, forces, etc.
    // By default a body is not fixed.
    body->SetBodyFixed(true);

    // =============================================================================================================
    // VISUAL ASSETS
    // =============================================================================================================

    // Add a visual asset to your body. It is only a matter of visualization, you can have a visual asset which is
    // completely different from the one you used to compute hydrodynamic loads.

    // Three default visual assets are available : sphere, cylinder and box shaped assets.
    body->AddSphereShape(20.); // radius = 20m

    // For adding your own mesh, specify a WaveFront .obj file name. MeshMagic software can help you convert other
    // mesh format into .obj : https://github.com/LHEEA/meshmagick.
    // body->AddMeshAsset("Ship.obj");

    // Select the color of your body, in the NAMED_COLOR enum (FrColors.h for more details).
    body->SetColor(IndianRed);

    // =============================================================================================================
    // PRINCIPAL INERTIAL PARAMETERS
    // =============================================================================================================

    // The inertia parameters, mass, inertia matrix and application point, are strongly linked, so we embedded them into
    // a single element: the inertia tensor (FrInertiaTensor). We chose to fix the application point to the body CoG by
    // convention. So, for setting mass and CoG position for a body, you need to use the SetInertiaTensor() method.

    // However you can use simple FrInertiaTensor constructors, if you just want to specify the mass, or the mass and
    // COG position. All inertia param not specified are implicitly set to null.
    double mass = 1000.; // in Kg
    Position COGPos(10.,-5.,-3.);
    //
    FrInertiaTensor_ InertiaTensor(mass);
    FrInertiaTensor_ InertiaTensor0(mass, COGPos, fc);

    // You can also choose to set the inertia coefficients, given at the COG frame (position and orientation, which may
    // not be the body orientation).
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;

    FrRotation_ COGOrientation;
    COGOrientation.SetCardanAngles_DEGREES(0.,0.,90.,fc); // COGFrame has a 90 yaw rotation, compared to the body reference frame.

    FrFrame_ COGFrame(COGPos,COGOrientation,fc); // creating COGFrame, from the COG position and orientation.
    FrInertiaTensor_ InertiaTensor2(mass,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,COGFrame,fc);

    // Finally, you may got your inertia coefficients expressed in a frame, auxFrame, that can be different from the
    // COGPosition. The inertia are then automatically transported to the CoG Position you specify by this following
    // constructor.
    FrFrame_ auxFrame; // defined from the position and orientation you got.

    FrInertiaTensor_ InertiaTensor3(mass,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,auxFrame,COGPos,fc);

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
    auto Node0 = body->NewNode(NodePos,fc);

    // Frame of the node, expressed in the body reference frame
    FrRotation_ NodeRot;
    NodeRot.SetNullRotation();
    FrFrame_ frameNode1(NodePos, NodeRot, fc);
    auto Node1 = body->NewNode(frameNode1);

    // =============================================================================================================
    // POSITIONS
    // =============================================================================================================

    // Different methods are available to set the position and orientation in the world reference frame. You can combine
    // some

}