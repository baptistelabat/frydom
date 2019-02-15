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

    /** Rotations can be expressed using several tools in FRyDoM : quaternions, cardan angles, rotation objects, etc.
     * We present in this demo the quaternions and rotations objects, with their constructors and associated methods.
     *
     */

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // =============================================================================================================
    // UNIT QUATERNION (add ref?)
    // =============================================================================================================
    // Unit quaternions are rotations by definition, since they have a unit norm. It is not to be confused with the
    // identity quaternion (1,0,0,0) which represent a null rotation. The following quaternion is set to the identity
    // quaternion using its basic constructor.
    FrUnitQuaternion_ IdQuaternion(1.,0.,0.,0.,fc);
    // You can also set a quaternion to the identity using the following method.
    IdQuaternion.SetNullRotation();

    // It is also possible to instantiate a quaternion, using four doubles, which is not normalized yet. The constructor
    // is then normalizing the quaternion to be sure it is a unit quaternion.
    bool is_non_Normalized = true;
    FrUnitQuaternion_ QuatNotNormalized(1.,2.,3.,4.,is_non_Normalized,fc);

    // If you are not familiar with quaternion, you can instantiate one by specifying the direction and the angle (in RAD)
    // of the rotation. The following rotation is for example a rotation around the z axis of an angle Pi.
    Direction QuatDir(0.,0.,1.);
    FrUnitQuaternion_ Quat(QuatDir,M_PI,fc);

    // Quaternions can also be defined, using set methods with the same arguments than the above constructors.
    Quat.Set(1.,0.,0.,0.,fc);
    Quat.Set(1.,2.,3.,4.,is_non_Normalized,fc);
    Quat.Set(QuatDir,M_PI,fc);

    // Remember that the quaternion can be composed, but their product is not commutable.
    QuatDir.Set(1.,0.,0.);
    FrUnitQuaternion_ QuatA(QuatDir,M_PI,fc);

    auto QuatB = Quat*QuatA;
    auto QuatC = QuatA*Quat;

    std::cout<<"Quat : "<<Quat<<std::endl;
    // Quaternion : q0 = 0, q1 = 0, q2 = 0, q3 = 1
    std::cout<<"QuatA : "<<QuatA<<std::endl;
    // Quaternion : q0 = 0, q1 = 1, q2 = 0, q3 = 0
    std::cout<<"QuatB : "<<QuatB<<std::endl;
    // Quaternion : q0 = 0, q1 = 0, q2 = 1, q3 = 0
    std::cout<<"QuatC : "<<QuatC<<std::endl;
    // Quaternion : q0 = 0, q1 = 0, q2 = -1, q3 = 0


    // =============================================================================================================
    // ROTATION
    // =============================================================================================================
    // A FrRotation object embedded a unit quaternion. It can then be instantiated using a quaternion or a direction and
    // angle.

    FrRotation_ Rotation(Quat);
    FrRotation_ RotationA(QuatDir,M_PI,fc);

    // A Null rotation can also be set, with the following method
    Rotation.SetNullRotation();

    // Amongst the set methods, there are those used for the previous contructors
    Rotation.Set(Quat);
    RotationA.SetAxisAngle(QuatDir,M_PI,fc);

    // However you can also specify Cardan sequence, using radian or degree angles.
    RotationA.SetCardanAngles_DEGREES(10,0.,90.,fc);

    // Rotation can also be composed, in the same manner as quaternions.
    auto RotationB = Rotation*RotationA;
    auto RotationC = RotationA*Rotation;

    std::cout<<"Rotation : "<<Rotation<<std::endl;
//    Rotation (cardan angles in deg, NWU convention) :
//    Cardan angle (deg) : phi   = 0; theta = -0; psi   = 180

    std::cout<<"RotationA : "<<RotationA<<std::endl;
//    Rotation (cardan angles in deg, NWU convention) :
//    Cardan angle (deg) : phi   = 10; theta = -0; psi   = 90

    std::cout<<"RotationB : "<<RotationB<<std::endl;
//    Rotation (cardan angles in deg, NWU convention) :
//    Cardan angle (deg) : phi   = 10; theta = 0; psi   = -90

    std::cout<<"RotationC : "<<RotationC<<std::endl;
//    Rotation (cardan angles in deg, NWU convention) :
//    Cardan angle (deg) : phi   = -10; theta = 0; psi   = -90


    // =============================================================================================================
    // FRAME
    // =============================================================================================================
    // A frame is composed of a Position and a Rotation and are used to locate components relatively to each other.
    // The parent frame is never defined and must be tracked from the context. Indeed, frames may also be thought as
    // transforms between frames as they are able to locate a "TO" frame with respect to a "FROM" frame.

    Position Pos(0.,0.,0.);

    // You can either instantiate a frame from a FrRotation object or a FrUnitQuaternion object.
    FrFrame_ Frame(Pos, Rotation, fc);
    FrFrame_ FrameA(Pos, Quat, fc);

    // You can also specify separately the position and the orientation of the frame.
    Frame.SetPosition(Pos, fc);

    Frame.SetRotation(Rotation);
    Frame.SetRotation(Quat);

    // You can specify no translation or rotation as well with the following methods
    Frame.SetNoTranslation();
    Frame.SetNoRotation();

    // You can compose frame, ie transform a frame using an other one.
    FrameA.SetPosition(Position(4.,5.,6.),fc);
    FrameA.SetRotation(QuatA);

    FrFrame_ FrameB(Position(1.,2.,3.),Quat,fc);


    // The '*' operator transforms a coordinate system, so
    // transformations can be represented with this syntax:
    //  new_frame = tr_frame * old_frame;
    // For a sequence of transformations, i.e. a chain of coordinate
    // systems, you can also write this (just like you would do with
    // a sequence of Denavitt-Hartemberg matrix multiplications!)
    //  new_frame = frame1to0 * frame2to1 * frame3to2 * old_frame;
    // This operation is not commutative.
    auto NewFrame = FrameB * FrameA;

    std::cout<<"FrameA :"<<FrameA<<std::endl;
//    Frame :
//    -------
//            Translation (m, In NWU): X = 4; Y = 5; Z = 6
//
//    Rotation (cardan angles in deg, NWU convention) :
//    Cardan angle (deg) : phi   = 180; theta = -0; psi   = 0

    std::cout<<"FrameB :"<<FrameB<<std::endl;
//    Frame :
//    -------
//            Translation (m, In NWU): X = 1; Y = 2; Z = 3
//
//    Rotation (cardan angles in deg, NWU convention) :
//    Cardan angle (deg) : phi   = 0; theta = -0; psi   = 180

    std::cout<<"NewFrame :"<<NewFrame<<std::endl;
//    Frame :
//    -------
//            Translation (m, In NWU): X = -3; Y = -3; Z = 9
//
//    Rotation (cardan angles in deg, NWU convention) :
//    Cardan angle (deg) : phi   = 180; theta = -0; psi   = 180



}
