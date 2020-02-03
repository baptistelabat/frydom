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

#include "gtest/gtest.h"

using namespace frydom;

// From Body To World
template <class Vector>
inline Vector& EasyRotate(Vector& vector, FRAME_CONVENTION fc) {
    Vector vecTem;
    if (IsNED(fc)) {internal::SwapFrameConvention<Vector>(vector);}
    vecTem[0] = vector[2];
    vecTem[1] = vector[0];
    vecTem[2] = vector[1];
    if (IsNED(fc)) {internal::SwapFrameConvention<Vector>(vecTem);}
    return vector = vecTem;
}

template <class Vector>
inline Vector EasyRotate(const Vector& vector, FRAME_CONVENTION fc) {
    Vector out = vector;
    return EasyRotate<Vector>(out,fc);
}

// From World To Body
template <class Vector>
inline Vector& EasyRotateInv(Vector& vector, FRAME_CONVENTION fc) {
    Vector vecTem;
    if (IsNED(fc)) {internal::SwapFrameConvention<Vector>(vector);}
    vecTem[0] = vector[1];
    vecTem[1] = vector[2];
    vecTem[2] = vector[0];
    if (IsNED(fc)) {internal::SwapFrameConvention<Vector>(vecTem);}
    return vector = vecTem;
}

template <class Vector>
inline Vector EasyRotateInv(const Vector& vector, FRAME_CONVENTION fc) {
    Vector out = vector;
    return EasyRotateInv<Vector>(out,fc);
}


void Test_AllGetPosition(const std::shared_ptr<FrBody> body,
        Position &RefPositionInWorld, Position &COGPositionInWorld, FRAME_CONVENTION in_fc,
        bool is_Orientation, FRAME_CONVENTION out_fc){

    if (out_fc != in_fc) {
        internal::SwapFrameConvention<Position>(RefPositionInWorld);
        internal::SwapFrameConvention<Position>(COGPositionInWorld);
    }

    // Test body reference frame position in world reference frame
    Position testPosition = body->GetPosition(out_fc) - RefPositionInWorld;
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())) {
        std::cout<<body->GetPosition(out_fc)<<std::endl;
        std::cout<<RefPositionInWorld<<std::endl;
    }

    //-----------------COG-----------------//
    // Test COG position in body reference frame
    Position TempPos = COGPositionInWorld - RefPositionInWorld;
    if (is_Orientation) EasyRotateInv(TempPos,out_fc);
    testPosition = body->GetCOG(out_fc) - TempPos;
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())) {
        std::cout<<body->GetCOG(out_fc)<<std::endl;
        std::cout<<TempPos<<std::endl;
    }

    // Test COG position in world reference frame
    testPosition = body->GetCOGPositionInWorld(out_fc) - COGPositionInWorld;
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())) {
        std::cout<<body->GetCOGPositionInWorld(out_fc)<<std::endl;
        std::cout<<COGPositionInWorld<<std::endl;
    }

    //-----------------Fixed Point-----------------//
    // Test for the getter for the local position of a point expressed in the world reference frame
    Position PointPositionInWorld(1., 5., 9.); // Position of a point, expressed in world reference frame
    TempPos = body->GetPointPositionInBody(PointPositionInWorld, out_fc);
    if (is_Orientation) EasyRotate(TempPos,out_fc);
    testPosition = TempPos - (PointPositionInWorld - RefPositionInWorld);
    EXPECT_TRUE(testPosition.isZero());

    Position PointPositionInBody(1.,5.,9.); // Position of a point, expressed in body reference frame
    // Test for the getter for the abs position of a point expressed in the body reference frame
    TempPos = PointPositionInBody;
    if (is_Orientation) EasyRotate(TempPos,out_fc);
    testPosition = body->GetPointPositionInWorld(PointPositionInBody, out_fc) - RefPositionInWorld - TempPos;
    EXPECT_TRUE(testPosition.isZero());

}

TEST(FrBodyTest,Position) {
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2., 3., 4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,COGPositionInBody,fc);
    body->SetInertiaTensor(InertiaTensor);

    Position COGPositionInWorld = RefPositionInWorld + COGPositionInBody;

    Test_AllGetPosition(body,RefPositionInWorld,COGPositionInWorld,fc,false,fc);
}

TEST(FrBodyTest,PositionNED) {

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2., 3., 4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,COGPositionInBody,NWU);
    body->SetInertiaTensor(InertiaTensor);

    Position COGPositionInWorld = RefPositionInWorld + COGPositionInBody;

    Test_AllGetPosition(body,RefPositionInWorld,COGPositionInWorld,NWU,false,NED);
}

TEST(FrBodyTest,Translation) {
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2., 3., 4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,COGPositionInBody,fc);
    body->SetInertiaTensor(InertiaTensor);

    Position COGPositionInWorld = RefPositionInWorld + COGPositionInBody;

    //-----------------Translate Body-----------------//
    Translation BodyTranslationInWorld(1.,4.,7.);
    if (IsNED(fc)) {internal::SwapFrameConvention<Translation>(BodyTranslationInWorld);}
    Position BodyPosition = RefPositionInWorld + BodyTranslationInWorld;
    Position COGPosition = COGPositionInWorld + BodyTranslationInWorld;

    //+++++Translate body reference frame+++++//
//    std::cout<<"+++++SetPosition"<<std::endl;
    body->SetPosition(RefPositionInWorld+BodyTranslationInWorld,fc);
    Test_AllGetPosition(body,BodyPosition,COGPosition,fc,false,fc);

    //+++++Translate body reference frame from fixed point+++++//
//    std::cout<<"+++++SetPointPosition"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    Position Point(4.,5.,6.); // Position of a point expressed in body reference frame
    body->SetPositionOfBodyPoint(Point, body->GetPointPositionInWorld(Point,fc) + BodyTranslationInWorld, fc);
    Test_AllGetPosition(body,BodyPosition,COGPosition,fc,false,fc);

    //+++++Translate body reference frame from translation expressed in world reference frame+++++//
//    std::cout<<"+++++TranslateInWorld"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInWorld(BodyTranslationInWorld,fc);
    Test_AllGetPosition(body,BodyPosition,COGPosition,fc,false,fc);

    //+++++Translate body reference frame from translation expressed in body reference frame+++++//
//    std::cout<<"+++++TranslateInBody"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInBody(BodyTranslationInWorld,fc);
    Test_AllGetPosition(body,BodyPosition,COGPosition,fc,false,fc);

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2 ;
    body->SetRotation(TotalRotation);

    //+++++Translate body reference frame from fixed point, with a rotation+++++//
    COGPosition = body->GetCOGPositionInWorld(fc);
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInBody(EasyRotateInv(BodyTranslationInWorld,fc),fc);
    Test_AllGetPosition(body,BodyPosition,COGPosition,fc,true,fc);

}


void Test_AllGetRotation(const std::shared_ptr<FrBody> body, const FrRotation &BodyRotationInWorld){

    // Test of the getter for the absolute orientation (expressed in the world reference frame)
    EXPECT_TRUE(body->GetRotation() == BodyRotationInWorld);

    // Test of the getter for the quaternion
    EXPECT_TRUE(body->GetQuaternion() == BodyRotationInWorld.GetQuaternion());

    // Test of the frame getter
    EXPECT_TRUE(body->GetFrame().GetRotation() == BodyRotationInWorld);
}

TEST(FrBodyTest,Orientation) {
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2., 3., 4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,COGPositionInBody,fc);
    body->SetInertiaTensor(InertiaTensor);
    Position COGPositionInWorld = RefPositionInWorld + COGPositionInBody;

    //-----------------Orientation-----------------//
    //+++++Set Rotation+++++//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    FrRotation BodyRotationInWorld;    BodyRotationInWorld.SetCardanAngles_DEGREES(1., 2., 3., NWU);
    body->SetRotation(BodyRotationInWorld);

    Test_AllGetRotation(body, BodyRotationInWorld);

//    //+++++Set Rotation, using Cardan angles+++++//
//    body->SetCardanAngles_DEGREES(1., 2., 3., fc);
//    EXPECT_TRUE(BodyRotationInWorld == body->GetRotation());

//    Test_AllGetRotation(body, BodyRotationInWorld);

    // Test of the frame position getter
    Position testPosition = body->GetFrame().GetPosition(fc) - RefPositionInWorld;
    EXPECT_TRUE(testPosition.isZero());

    //+++++Set Frame+++++//
    FrFrame RefFrame(RefPositionInWorld,BodyRotationInWorld,fc);
    body->SetFrame(RefFrame);

    Test_AllGetRotation(body, BodyRotationInWorld);
    // Test of the frame position getter
    testPosition = body->GetFrame().GetPosition(fc) - RefFrame.GetPosition(fc);
    EXPECT_TRUE(testPosition.isZero());

    //-----------------Rotation-----------------//
    //+++++Rotate+++++//
    FrRotation NewRotation; NewRotation.SetCardanAngles_DEGREES(4.,8.,3.,NWU);
    body->Rotate(NewRotation);

    BodyRotationInWorld *= NewRotation;
    Test_AllGetRotation(body, BodyRotationInWorld);

    //+++++RotateAroundCOG+++++//
    FrRotation RotationAroundCOG; RotationAroundCOG.SetCardanAngles_DEGREES(8.,6.,2.,NWU);
    body->RotateAroundCOG(RotationAroundCOG,fc);

    BodyRotationInWorld *= RotationAroundCOG;
    Test_AllGetRotation(body, BodyRotationInWorld);

    //+++++RotateAroundPointInWorld+++++//
    Position PointInWorld(5.,4.,8.);
    FrRotation RotationAroundPointInWorld; RotationAroundPointInWorld.SetCardanAngles_DEGREES(1.,9.,7.,NWU);
    body->RotateAroundPointInWorld(RotationAroundPointInWorld, PointInWorld, fc);

    BodyRotationInWorld *= RotationAroundPointInWorld;
    Test_AllGetRotation(body, BodyRotationInWorld);

    //+++++RotateAroundPointInBody+++++//
    Position PointInBody(5.,4.,8.);
    FrRotation RotationAroundPointInBody; RotationAroundPointInBody.SetCardanAngles_DEGREES(1.,9.,7.,NWU);
    body->RotateAroundPointInWorld(RotationAroundPointInBody, PointInBody, fc);

    BodyRotationInWorld *= RotationAroundPointInBody;
    Test_AllGetRotation(body, BodyRotationInWorld);

    //+++++Test Position after Rotating around a Point+++++//
    // Init the Frame to original Position and Rotation
    BodyRotationInWorld.SetCardanAngles_DEGREES(0., 0., 0., fc);
    RefFrame.SetPosition(RefPositionInWorld,fc);
    RefFrame.SetRotation(BodyRotationInWorld);
    body->SetFrame(RefFrame);

    Test_AllGetRotation(body, BodyRotationInWorld);

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2 ;

    // Rotate around the RefPositionInWorld, using RotateAroundPointInWorld, to check the position
    body->RotateAroundPointInWorld(TotalRotation, RefPositionInWorld, fc);

    Position OrigAbsCOGPos = RefPositionInWorld + EasyRotate(COGPositionInBody,fc);
    Test_AllGetPosition(body, RefPositionInWorld, OrigAbsCOGPos, fc, true, fc);
}

TEST(FrBodyTest,PositionWithOrientation){
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position RefPositionInWorld(1.,2.,3.);
    body->SetPosition(RefPositionInWorld,fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2.,3.,4.);
    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,COGPositionInBody,fc);
    body->SetInertiaTensor(InertiaTensor);

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2 ;
    body->SetRotation(TotalRotation);

    Position OrigAbsCOGPos = RefPositionInWorld + EasyRotate(COGPositionInBody,fc);

    Test_AllGetPosition(body, RefPositionInWorld, OrigAbsCOGPos, fc, true, fc);

}

void Test_GetAngularVelocity(const std::shared_ptr<FrBody> body,
                             const AngularVelocity &BodyAngularVelocityInWorld,
                             bool is_orientation, FRAME_CONVENTION fc){

    AngularVelocity testAngularVelocity;

    // Test Angular Velocity getter, expressed in world reference frame
    testAngularVelocity = body->GetAngularVelocityInWorld(fc) - BodyAngularVelocityInWorld;
    EXPECT_TRUE(testAngularVelocity.isZero());

    // Test Angular Velocity getter, expressed in body reference frame
    testAngularVelocity = body->GetAngularVelocityInBody(fc);
    if (is_orientation) EasyRotate(testAngularVelocity,fc);
    testAngularVelocity -= BodyAngularVelocityInWorld;
    EXPECT_TRUE(testAngularVelocity.isZero());
}

TEST(FrBodyTest,AngularVelocity) {
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    AngularVelocity BodyAngularVelocityInBody, BodyAngularVelocityInWorld;

    //+++++Test Angular Velocity Setters and Getters without any body reference frame rotation+++++//
    // Test Angular Velocity setter, expressed in world reference frame
    BodyAngularVelocityInWorld.Set(1.,2.,3.);
    body->SetAngularVelocityInWorld(BodyAngularVelocityInWorld,fc);
//    std::cout<<"SetAngularVelocityInWorld"<<std::endl;
    Test_GetAngularVelocity(body,BodyAngularVelocityInWorld,false,fc);


    // Test Angular Velocity setter, expressed in body reference frame
    BodyAngularVelocityInBody.Set(4.,5.,6.);
    body->SetAngularVelocityInBody(BodyAngularVelocityInBody,fc);
//    std::cout<<"SetAngularVelocityInBody"<<std::endl;
    Test_GetAngularVelocity(body,BodyAngularVelocityInBody,false,fc);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

    //+++++Test Angular Velocity Setters and Getters with a body reference frame rotation+++++//
    // Test Angular Velocity setter, expressed in world reference frame
    BodyAngularVelocityInWorld.Set(1.,2.,3.);
    body->SetAngularVelocityInWorld(BodyAngularVelocityInWorld,fc);
//    std::cout<<"SetAngularVelocityInWorld, with a rotation"<<std::endl;
    Test_GetAngularVelocity(body,BodyAngularVelocityInWorld,true,fc);


    // Test Angular Velocity setter, expressed in body reference frame
    BodyAngularVelocityInBody.Set(4.,5.,6.);
    body->SetAngularVelocityInBody(BodyAngularVelocityInBody,fc);
//    std::cout<<"SetAngularVelocityInBody, with a rotation"<<std::endl;
    BodyAngularVelocityInWorld = EasyRotate(BodyAngularVelocityInBody,fc);
    Test_GetAngularVelocity(body,BodyAngularVelocityInWorld,true,fc);


}


void Test_AllGetVelocity(const std::shared_ptr<FrBody> body,
                         const Velocity &VelocityInWorld, const AngularVelocity &AngularVelocityInWorld,
                         bool is_Rotation, FRAME_CONVENTION fc) {

    Velocity VelocityInBody = VelocityInWorld;
    if (is_Rotation) EasyRotateInv(VelocityInBody,fc);


    // Test getter for the body velocity, expressed in the world reference frame
    Velocity testVelocity;
    testVelocity = body->GetLinearVelocityInWorld(fc) - VelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())) {
        std::cout<<VelocityInWorld<<std::endl;
        std::cout<<body->GetLinearVelocityInWorld(fc)<<std::endl;
    }
    // Test getter for the body velocity, expressed in the body reference frame
    testVelocity = body->GetVelocityInBody(fc) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());
//    std::cout<<body->GetVelocityInBody(fc)<<std::endl;


    //-----------------COG Velocity-----------------//
    // Test Getter for the COG velocity expressed in the world reference frame
    testVelocity = body->GetCOGLinearVelocityInWorld(fc) - VelocityInWorld;
    testVelocity -= AngularVelocityInWorld.cross(body->GetCOGPositionInWorld(fc) - body->GetPosition(fc));
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())) {
        std::cout<<VelocityInWorld +
        AngularVelocityInWorld.cross(body->GetCOGPositionInWorld(fc) - body->GetPosition(fc))<<std::endl;
        std::cout<<body->GetCOGLinearVelocityInWorld(fc)<<std::endl;
    }

    // Test Getter for the COG velocity expressed in the body reference frame
    testVelocity = - AngularVelocityInWorld.cross(body->GetCOGPositionInWorld(fc) - body->GetPosition(fc));
    if (is_Rotation) testVelocity = EasyRotateInv(testVelocity,fc);
    testVelocity += body->GetCOGVelocityInBody(fc) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());


    //-----------------PointInBody-----------------//
    Position PointInBody(5.,6.,7.);    Position PointInWorld(7.,6.,5.);
    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in body reference frame
    testVelocity = body->GetVelocityInWorldAtPointInBody(PointInBody, fc) - VelocityInWorld;
    testVelocity -= AngularVelocityInWorld.cross(body->GetPointPositionInWorld(PointInBody, fc) - body->GetPosition(fc));
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in world reference frame
    testVelocity = body->GetVelocityInWorldAtPointInWorld(PointInWorld, fc) - VelocityInWorld;
    testVelocity -= AngularVelocityInWorld.cross(PointInWorld - body->GetPosition(fc));
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in body reference frame
    testVelocity = - AngularVelocityInWorld.cross(body->GetPointPositionInWorld(PointInBody, fc) - body->GetPosition(fc));
    if (is_Rotation) testVelocity = EasyRotateInv(testVelocity,fc);
    testVelocity += body->GetVelocityInBodyAtPointInBody(PointInBody, fc) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())) {
        std::cout<<VelocityInBody<<std::endl;
        std::cout<<body->GetVelocityInBody(fc)<<std::endl;
        std::cout<<body->GetVelocityInBodyAtPointInBody(PointInBody, fc)<<std::endl;
    }

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in world reference frame
    testVelocity = - AngularVelocityInWorld.cross(PointInWorld - body->GetPosition(fc));
    if (is_Rotation) testVelocity = EasyRotateInv(testVelocity,fc);
    testVelocity += body->GetVelocityInBodyAtPointInWorld(PointInWorld, fc) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());
}

TEST(FrBodyTest,TranslationalVelocity){
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,OrigLocalCOGPos,fc);
    body->SetInertiaTensor(InertiaTensor);

    //-----------------Angular Velocity-----------------//
    AngularVelocity BodyAngularVelocity(0,0,0);
    bool is_orientation = false;

    //-----------------Velocity Setters-----------------//
    //+++++COG Frame Velocity Setters+++++//
    // Set the body velocity, expressed in the world reference frame
    Velocity VelocityInWorld(1.,1.,1.);
    body->SetVelocityInWorldNoRotation(VelocityInWorld,fc);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    // Set the body velocity, expressed in the body reference frame
    Velocity VelocityInBody(1.,6.,4.);
    body->SetVelocityInBodyNoRotation(VelocityInBody,fc);
    Test_AllGetVelocity(body,VelocityInBody, BodyAngularVelocity, is_orientation,fc);

    // Set the body generalized velocity, expressed in the world reference frame
    body->SetGeneralizedVelocityInWorld(VelocityInWorld, BodyAngularVelocity, fc);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    // Set the body generalized velocity, expressed in the body reference frame
    body->SetGeneralizedVelocityInBody(VelocityInBody, BodyAngularVelocity, fc);
    Test_AllGetVelocity(body,VelocityInBody, BodyAngularVelocity, is_orientation,fc);

    //+++++Point Velocity Setters+++++//
    Position PointInWorld(5.,6.,7.); Position PointInBody(7.,6.,5.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,fc);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInWorld"<<std::endl;
    Velocity VelocityInWorldAtPointInWorld(3.,1.,5.);
    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,BodyAngularVelocity,fc);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInWorld, BodyAngularVelocity, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInBody"<<std::endl;
    Velocity VelocityInWorldAtPointInBody(2.,1.,8.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,BodyAngularVelocity,fc);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInBody, BodyAngularVelocity, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInWorld"<<std::endl;
    Velocity VelocityInBodyAtPointInWorld(8.,1.,9.);
    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,BodyAngularVelocity,fc);

    Test_AllGetVelocity(body, VelocityInBodyAtPointInWorld, BodyAngularVelocity, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInBody"<<std::endl;
    Velocity VelocityInBodyAtPointInBody(1.,1.,5.);
    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,BodyAngularVelocity,fc);

    Test_AllGetVelocity(body, VelocityInBodyAtPointInBody, BodyAngularVelocity, is_orientation,fc);

}


TEST(FrBodyTest,TranslationalVelocityWithOrientation){
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,fc);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

    bool is_orientation = true;

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,OrigLocalCOGPos,fc);
    body->SetInertiaTensor(InertiaTensor);

    //-----------------Angular Velocity-----------------//
    AngularVelocity BodyAngularVelocity(0,0,0);

    //-----------------Velocity Setters-----------------//
    //+++++COG Frame Velocity Setters+++++//
    // Set the body velocity, expressed in the world reference frame
    Velocity VelocityInWorld(1.,1.,1.);
//    std::cout<<"SetVelocityInWorldNoRotation"<<std::endl;
    body->SetVelocityInWorldNoRotation(VelocityInWorld,fc);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    // Set the body velocity, expressed in the body reference frame
    const Velocity VelocityInBody(1.,6.,4.);
    VelocityInWorld = EasyRotate(VelocityInBody,fc);
//    std::cout<<"SetVelocityInBodyNoRotation"<<std::endl;
    body->SetVelocityInBodyNoRotation(VelocityInBody,fc);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    // Set the body generalized velocity, expressed in the world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorld"<<std::endl;
    body->SetGeneralizedVelocityInWorld(VelocityInWorld, BodyAngularVelocity, fc);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    // Set the body generalized velocity, expressed in the body reference frame
//    std::cout<<"SetGeneralizedVelocityInBody"<<std::endl;
    body->SetGeneralizedVelocityInBody(VelocityInBody, BodyAngularVelocity, fc);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    //+++++Point Velocity Setters+++++//
    Position PointInWorld(5.,6.,7.); Position PointInBody(7.,6.,5.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,fc);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInWorld"<<std::endl;
    Velocity VelocityInWorldAtPointInWorld(3.,1.,5.);
    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,BodyAngularVelocity,fc);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInWorld, BodyAngularVelocity, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInBody"<<std::endl;
    Velocity VelocityInWorldAtPointInBody(2.,1.,8.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,BodyAngularVelocity,fc);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInBody, BodyAngularVelocity, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInWorld"<<std::endl;
    const Velocity VelocityInBodyAtPointInWorld(8.,1.,9.);
    VelocityInWorldAtPointInWorld = EasyRotate(VelocityInBodyAtPointInWorld,fc);
    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,BodyAngularVelocity,fc);
    Test_AllGetVelocity(body, VelocityInWorldAtPointInWorld, BodyAngularVelocity, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in body reference frame
    const Velocity VelocityInBodyAtPointInBody(1.,1.,5.);
    VelocityInWorldAtPointInBody = EasyRotate(VelocityInBodyAtPointInBody,fc);
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInBody"<<std::endl;
    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,BodyAngularVelocity,fc);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInBody, BodyAngularVelocity, is_orientation,fc);
}


TEST(FrBodyTest,TranslationalVelocityWithAngularVelocity){
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,OrigLocalCOGPos,fc);
    body->SetInertiaTensor(InertiaTensor);

    //-----------------Angular Velocity-----------------//
    AngularVelocity BodyAngularVelocity(1,5,9);
    bool is_orientation = false;

    //-----------------Velocity Setters-----------------//
    //+++++COG Frame Velocity Setters+++++//
    // Set the body velocity, expressed in the world reference frame
    Velocity VelocityInWorld(1.,1.,1.);

    // Set the body velocity, expressed in the body reference frame
    Velocity VelocityInBody(1.,6.,4.);

    // Set the body generalized velocity, expressed in the world reference frame
    body->SetGeneralizedVelocityInWorld(VelocityInWorld, BodyAngularVelocity, fc);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    // Set the body generalized velocity, expressed in the body reference frame
    body->SetGeneralizedVelocityInBody(VelocityInBody, BodyAngularVelocity, fc);
    Test_AllGetVelocity(body,VelocityInBody, BodyAngularVelocity, is_orientation,fc);

    //+++++Point Velocity Setters+++++//
    Position PointInWorld(5.,6.,7.); Position PointInBody(7.,6.,5.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,fc);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInWorld"<<std::endl;
    Velocity VelocityInWorldAtPointInWorld(3.,1.,5.);
    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,BodyAngularVelocity,fc);

    VelocityInWorld = VelocityInWorldAtPointInWorld - BodyAngularVelocity.cross(PointInWorld - body->GetPosition(fc));
    Test_AllGetVelocity(body, VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInBody"<<std::endl;
    Velocity VelocityInWorldAtPointInBody(2.,1.,8.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,BodyAngularVelocity,fc);

    VelocityInWorld = VelocityInWorldAtPointInBody -
            BodyAngularVelocity.cross(body->GetPointPositionInWorld(PointInBody,fc) - body->GetPosition(fc));
    Test_AllGetVelocity(body, VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInWorld"<<std::endl;
    Velocity VelocityInBodyAtPointInWorld(8.,1.,9.);
    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,BodyAngularVelocity,fc);

    VelocityInWorld = VelocityInBodyAtPointInWorld - BodyAngularVelocity.cross(PointInWorld - body->GetPosition(fc));
    Test_AllGetVelocity(body, VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInBody"<<std::endl;
    Velocity VelocityInBodyAtPointInBody(1.,1.,5.);
    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,BodyAngularVelocity,fc);

    VelocityInWorld = VelocityInBodyAtPointInBody -
                      BodyAngularVelocity.cross(body->GetPointPositionInWorld(PointInBody,fc) - body->GetPosition(fc));
    Test_AllGetVelocity(body, VelocityInWorld, BodyAngularVelocity, is_orientation,fc);

}


TEST(FrBodyTest,TranslationalVelocityWithAngularVelocityAndOrientation){
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,OrigLocalCOGPos,fc);
    body->SetInertiaTensor(InertiaTensor);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

    bool is_orientation = true;

    //-----------------Angular Velocity-----------------//
    const AngularVelocity AngularVelocityInWorld(1,5,9);
    AngularVelocity AngularVelocityInBody = EasyRotateInv(AngularVelocityInWorld,fc);

    //-----------------Velocity Setters-----------------//
    //+++++COG Frame Velocity Setters+++++//
    // Set the body velocity, expressed in the world reference frame
    Velocity VelocityInWorld(1.,3.,8.);

    // Set the body velocity, expressed in the body reference frame
    const Velocity VelocityInBody(1.,6.,4.);

    // Set the body generalized velocity, expressed in the world reference frame
    body->SetGeneralizedVelocityInWorld(VelocityInWorld, AngularVelocityInWorld, fc);
    Test_AllGetVelocity(body,VelocityInWorld, AngularVelocityInWorld, is_orientation,fc);

    // Set the body generalized velocity, expressed in the body reference frame
    body->SetGeneralizedVelocityInBody(VelocityInBody, AngularVelocityInBody, fc);

    VelocityInWorld = EasyRotate(VelocityInBody,fc);
    Test_AllGetVelocity(body,VelocityInWorld, AngularVelocityInWorld, is_orientation,fc);

    //+++++Point Velocity Setters+++++//
    Position PointInWorld(5.,6.,7.); Position PointInBody(7.,6.,5.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,fc);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInWorld"<<std::endl;
    Velocity VelocityInWorldAtPointInWorld(3.,1.,5.);
    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,AngularVelocityInWorld,fc);

    VelocityInWorld = VelocityInWorldAtPointInWorld - AngularVelocityInWorld.cross(PointInWorld - body->GetPosition(fc));
    Test_AllGetVelocity(body, VelocityInWorld, AngularVelocityInWorld, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInBody"<<std::endl;
    Velocity VelocityInWorldAtPointInBody(2.,1.,8.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,AngularVelocityInWorld,fc);

    VelocityInWorld = VelocityInWorldAtPointInBody -
            AngularVelocityInWorld.cross(body->GetPointPositionInWorld(PointInBody,fc) - body->GetPosition(fc));
    Test_AllGetVelocity(body, VelocityInWorld, AngularVelocityInWorld, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInWorld"<<std::endl;
    const Velocity VelocityInBodyAtPointInWorld(8.,1.,9.);
    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,AngularVelocityInBody,fc);

    VelocityInWorld = EasyRotate(VelocityInBodyAtPointInWorld,fc) - AngularVelocityInWorld.cross(PointInWorld - body->GetPosition(fc));
    Test_AllGetVelocity(body, VelocityInWorld, AngularVelocityInWorld, is_orientation,fc);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInBody"<<std::endl;
    const Velocity VelocityInBodyAtPointInBody(1.,1.,5.);
    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,AngularVelocityInBody,fc);

    VelocityInWorld = EasyRotate(VelocityInBodyAtPointInBody,fc) -
            AngularVelocityInWorld.cross(body->GetPointPositionInWorld(PointInBody,fc) - body->GetPosition(fc));
    Test_AllGetVelocity(body, VelocityInWorld, AngularVelocityInWorld, is_orientation,fc);

}



void Test_GetAngularAcceleration(const std::shared_ptr<FrBody> body,
                             const AngularAcceleration &BodyAngularAccelerationInWorld,
                             bool is_orientation, FRAME_CONVENTION fc){

    AngularVelocity testAngularAcceleration;

    // Test Angular Velocity getter, expressed in world reference frame
    testAngularAcceleration = body->GetAngularAccelerationInWorld(fc) - BodyAngularAccelerationInWorld;
    EXPECT_TRUE(testAngularAcceleration.isZero());
    if (not(testAngularAcceleration.isZero())){
        std::cout<<body->GetAngularAccelerationInWorld(fc)<<std::endl;
        std::cout<<BodyAngularAccelerationInWorld<<std::endl;
    }

    // Test Angular Velocity getter, expressed in body reference frame
    testAngularAcceleration = body->GetAngularAccelerationInBody(fc);
    if (is_orientation) EasyRotate(testAngularAcceleration,fc);
    testAngularAcceleration -= BodyAngularAccelerationInWorld;
    EXPECT_TRUE(testAngularAcceleration.isZero());
}

TEST(FrBodyTest,AngularAcceleration) {
    FRAME_CONVENTION fc = NED;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    AngularAcceleration BodyAngularAccelerationInBody, BodyAngularAccelerationInWorld;

    //+++++Test Angular Acceleration Setters and Getters without any body reference frame rotation+++++//
    // Test Angular Acceleration setter, expressed in world reference frame
    BodyAngularAccelerationInWorld.Set(1.,2.,3.);
    body->SetAngularAccelerationInWorld(BodyAngularAccelerationInWorld,fc);
//    std::cout<<"SetAngularAccelerationInWorld"<<std::endl;
    Test_GetAngularAcceleration(body,BodyAngularAccelerationInWorld,false,fc);


    // Test Angular Acceleration setter, expressed in body reference frame
    BodyAngularAccelerationInBody.Set(4.,5.,6.);
    body->SetAngularAccelerationInBody(BodyAngularAccelerationInBody,fc);
//    std::cout<<"SetAngularAccelerationInBody"<<std::endl;
    Test_GetAngularAcceleration(body,BodyAngularAccelerationInBody,false,fc);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

    //+++++Test Angular Acceleration Setters and Getters with a body reference frame rotation+++++//
    // Test Angular Acceleration setter, expressed in world reference frame
    BodyAngularAccelerationInWorld.Set(1.,2.,3.);
    body->SetAngularAccelerationInWorld(BodyAngularAccelerationInWorld,fc);
//    std::cout<<"SetAngularAccelerationInWorld, with a rotation"<<std::endl;
    Test_GetAngularAcceleration(body,BodyAngularAccelerationInWorld,true,fc);


    // Test Angular Acceleration setter, expressed in body reference frame
    BodyAngularAccelerationInBody.Set(4.,5.,6.);
    body->SetAngularAccelerationInBody(BodyAngularAccelerationInBody,fc);
//    std::cout<<"SetAngularAccelerationInBody, with a rotation"<<std::endl;
    BodyAngularAccelerationInWorld = EasyRotate(BodyAngularAccelerationInBody,fc);
    Test_GetAngularAcceleration(body,BodyAngularAccelerationInWorld,true,fc);


}



void Test_AllGetAcceleration(const std::shared_ptr<FrBody> body,
                             const Acceleration &COGAccelerationInWorld,
                             const AngularAcceleration &AngularAccelerationInWorld,
                             const AngularVelocity &AngularVelocityInWorld,
                             bool is_Rotation, FRAME_CONVENTION fc){

    Acceleration COGAccelerationInBody = COGAccelerationInWorld;
    if (is_Rotation) EasyRotateInv(COGAccelerationInBody,fc);

    Position OGInWorld = body->GetCOGPositionInWorld(fc) - body->GetPosition(fc);

    //-----------------COG Acceleration-----------------//
    // Test Getter for the COG Acceleration expressed in the world reference frame
    Acceleration testAcceleration = body->GetCOGLinearAccelerationInWorld(fc) - COGAccelerationInWorld;
    EXPECT_TRUE(testAcceleration.isZero());
    if (not(testAcceleration.isZero())) {
        std::cout<<body->GetCOGLinearAccelerationInWorld(fc)<<std::endl;
        std::cout<<COGAccelerationInWorld<<std::endl;
    }

    // Test Getter for the COG Acceleration expressed in the body reference frame
    testAcceleration = body->GetCOGAccelerationInBody(fc) - COGAccelerationInBody;
    EXPECT_TRUE(testAcceleration.isZero());
    if (not(testAcceleration.isZero())) {
        std::cout<<body->GetCOGAccelerationInBody(fc)<<std::endl;
        std::cout<<COGAccelerationInBody<<std::endl;
    }

    //-----------------PointInBody-----------------//
    Position PointInBody(5.,6.,7.);    Position PointInWorld(7.,6.,5.);

    // Test Getter for the Acceleration expressed in the world reference frame, at a Point expressed in body reference frame
    Position PG = body->GetPointPositionInWorld(PointInBody,fc) - body->GetCOGPositionInWorld(fc);
    Acceleration AccelerationToCompare = COGAccelerationInWorld + AngularAccelerationInWorld.cross(PG)
                            + AngularVelocityInWorld.cross(AngularVelocityInWorld.cross(PG)) ;
    testAcceleration = body->GetAccelerationInWorldAtPointInBody(PointInBody, fc) - AccelerationToCompare;
    EXPECT_TRUE(testAcceleration.isZero());
    if (not(testAcceleration.isZero())) {
        std::cout<<body->GetAccelerationInWorldAtPointInBody(PointInBody, fc)<<std::endl;
        std::cout<<AccelerationToCompare<<std::endl;
    }

    // Test Getter for the Acceleration expressed in the world reference frame, at a Point expressed in world reference frame
    PG = PointInWorld - body->GetCOGPositionInWorld(fc);
    AccelerationToCompare = COGAccelerationInWorld + AngularAccelerationInWorld.cross(PG)
                            + AngularVelocityInWorld.cross(AngularVelocityInWorld.cross(PG)) ;
    testAcceleration = body->GetAccelerationInWorldAtPointInWorld(PointInWorld, fc) - AccelerationToCompare;
    EXPECT_TRUE(testAcceleration.isZero());
    if (not(testAcceleration.isZero())) {
        std::cout<<body->GetAccelerationInWorldAtPointInWorld(PointInBody, fc)<<std::endl;
        std::cout<<AccelerationToCompare<<std::endl;
    }

    // Test Getter for the Acceleration expressed in the body reference frame, at a Point expressed in body reference frame
    PG = body->GetPointPositionInWorld(PointInBody,fc) - body->GetCOGPositionInWorld(fc);
    AccelerationToCompare = COGAccelerationInWorld + AngularAccelerationInWorld.cross(PG)
                            + AngularVelocityInWorld.cross(AngularVelocityInWorld.cross(PG));
    testAcceleration = body->GetAccelerationInBodyAtPointInBody(PointInBody, fc);
    if (is_Rotation) testAcceleration = EasyRotate(testAcceleration,fc);
    testAcceleration -= AccelerationToCompare;
    EXPECT_TRUE(testAcceleration.isZero());
    if (not(testAcceleration.isZero())) {
        std::cout<<body->GetAccelerationInBodyAtPointInBody(PointInBody, fc)<<std::endl;
        std::cout<<AccelerationToCompare<<std::endl;
    }

    // Test Getter for the Acceleration expressed in the body reference frame, at a Point expressed in world reference frame
    PG = PointInWorld - body->GetCOGPositionInWorld(fc);
    AccelerationToCompare = COGAccelerationInWorld + AngularAccelerationInWorld.cross(PG)
                            + AngularVelocityInWorld.cross(AngularVelocityInWorld.cross(PG)) ;
    testAcceleration = body->GetAccelerationInBodyAtPointInWorld(PointInWorld, fc);
    if (is_Rotation) testAcceleration = EasyRotate(testAcceleration,fc);
    testAcceleration -= AccelerationToCompare;
    EXPECT_TRUE(testAcceleration.isZero());
    if (not(testAcceleration.isZero())) {
        std::cout<<body->GetAccelerationInBodyAtPointInWorld(PointInWorld, fc)<<std::endl;
        std::cout<<AccelerationToCompare<<std::endl;
    }


}


TEST(FrBodyTest,Acceleration){
    FRAME_CONVENTION fc = NED;
    bool is_orientation = false;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos, fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,OrigLocalCOGPos,fc);
    body->SetInertiaTensor(InertiaTensor);

    //+++++SetAccelerationInWorldNoRotation+++++//
    Acceleration AccelerationInWorld(5.,6.,9.);
    body->SetAccelerationInWorldNoRotation(AccelerationInWorld, fc);
    AngularAcceleration AngularAccelerationInWorld = body->GetAngularAccelerationInWorld(fc);
    AngularVelocity AngularVelocityInWorld = body->GetAngularVelocityInWorld(fc);

    Test_AllGetAcceleration(body, AccelerationInWorld, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);

    //+++++SetAccelerationInBodyNoRotation+++++//
    Acceleration AccelerationInBody(7.,2.,4.);
    body->SetAccelerationInBodyNoRotation(AccelerationInBody, fc);

    Test_AllGetAcceleration(body, AccelerationInBody, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);

    //+++++SetGeneralizedAccelerationInBodyAtCOG+++++//
    body->SetGeneralizedAccelerationInBodyAtCOG(AccelerationInBody, AngularAccelerationInWorld, fc);

    Test_AllGetAcceleration(body, AccelerationInBody, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);

}

TEST(FrBodyTest,AccelerationWithOrientation){
    FRAME_CONVENTION fc = NED;
    bool is_orientation = true;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos, fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,OrigLocalCOGPos,fc);
    body->SetInertiaTensor(InertiaTensor);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

    //+++++SetAccelerationInWorldNoRotation+++++//
    Acceleration AccelerationInWorld(5.,6.,9.);
    body->SetAccelerationInWorldNoRotation(AccelerationInWorld, fc);
    auto AngularAccelerationInWorld = body->GetAngularAccelerationInWorld(fc);
    auto AngularVelocityInWorld = body->GetAngularVelocityInWorld(fc);

    Test_AllGetAcceleration(body, AccelerationInWorld, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);

    //+++++SetAccelerationInBodyNoRotation+++++//
    const Acceleration AccelerationInBody(7.,2.,4.);
    body->SetAccelerationInBodyNoRotation(AccelerationInBody, fc);

    AccelerationInWorld = EasyRotate(AccelerationInBody,fc);
    Test_AllGetAcceleration(body, AccelerationInWorld, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);


    //+++++SetGeneralizedAccelerationInBodyAtCOG+++++//
    body->SetGeneralizedAccelerationInBodyAtCOG(AccelerationInBody, AngularAccelerationInWorld, fc);

    Test_AllGetAcceleration(body, AccelerationInWorld, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);
}

TEST(FrBodyTest,AccelerationWithAngularVelocityAndAcceleration){
    FRAME_CONVENTION fc = NED;
    bool is_orientation = false;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos, fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(6.,5.,4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,OrigLocalCOGPos,fc);
    body->SetInertiaTensor(InertiaTensor);

    //-----------------Generalized Velocity, imposed at COG-----------------//
    Velocity VelocityInWorld(2.,8.,6.);
    AngularVelocity AngularVelocityInWorld(1.,5.,9.);
//    AngularVelocity AngularVelocityInWorld(0.,0.,0.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(OrigLocalCOGPos, VelocityInWorld, AngularVelocityInWorld, fc);

    //-----------------Generalized Acceleration, imposed at COG-----------------//
    // If we use these 2 methods (SetAccelerationInWorldNoRotation followed by SetAngularAccelerationInWorld),
    // the linear velocity is imposed on the COG !!!
    //+++++SetAccelerationInWorldNoRotation+++++//
    Acceleration AccelerationInWorld(5.,6.,9.);
    body->SetAccelerationInWorldNoRotation(AccelerationInWorld, fc);

    //-----------------Angular Acceleration-----------------//
    AngularAcceleration AngularAccelerationInWorld(5.,8.,4.);
    body->SetAngularAccelerationInWorld(AngularAccelerationInWorld,fc);

    Test_AllGetAcceleration(body, AccelerationInWorld, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);

    //+++++SetGeneralizedAccelerationInBodyAtCOG+++++//
    body->SetGeneralizedAccelerationInWorldAtCOG(AccelerationInWorld, AngularAccelerationInWorld, fc);

//    AccelerationInWorld = AccelerationInWorld + AngularAccelerationInWorld.cross(body->GetCOGPositionInWorld(fc))
//            + AngularVelocityInWorld.cross(AngularVelocityInWorld.cross(body->GetCOGPositionInWorld(fc)));

    Test_AllGetAcceleration(body, AccelerationInWorld, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);

}

TEST(FrBodyTest,AccelerationWithOrientationAndAngularVelocityAndAcceleration){
    FRAME_CONVENTION fc = NED;
    bool is_orientation = true;

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos, fc);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(6.,5.,4.);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,OrigLocalCOGPos,fc);
    body->SetInertiaTensor(InertiaTensor);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

    //-----------------Generalized Velocity, imposed at COG-----------------//
    Velocity VelocityInWorld(2.,8.,6.);
    AngularVelocity AngularVelocityInWorld(1.,5.,9.);
//    AngularVelocity AngularVelocityInWorld(0.,0.,0.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(OrigLocalCOGPos, VelocityInWorld, AngularVelocityInWorld, fc);

    //-----------------Generalized Acceleration, imposed at COG-----------------//
    // If we use these 2 methods (SetAccelerationInWorldNoRotation followed by SetAngularAccelerationInWorld),
    // the linear velocity is imposed on the COG !!!
    //+++++SetAccelerationInWorldNoRotation+++++//
    Acceleration AccelerationInWorld(5.,6.,9.);
    body->SetAccelerationInWorldNoRotation(AccelerationInWorld, fc);

    //-----------------Angular Acceleration-----------------//
    AngularAcceleration AngularAccelerationInWorld(5.,8.,4.);
    body->SetAngularAccelerationInWorld(AngularAccelerationInWorld,fc);

    Test_AllGetAcceleration(body, AccelerationInWorld, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);

    //+++++SetGeneralizedAccelerationInBodyAtCOG+++++//
    body->SetGeneralizedAccelerationInWorldAtCOG(AccelerationInWorld, AngularAccelerationInWorld, fc);

    Test_AllGetAcceleration(body, AccelerationInWorld, AngularAccelerationInWorld, AngularVelocityInWorld, is_orientation, fc);

}

TEST(FrBodyTest,ProjectVectorMethods){

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    Position NWUWorldPos(1,2,3);

    Position NWUBodyPos = body->ProjectVectorInBody(NWUWorldPos,NWU);
    Position testPosition = NWUBodyPos - NWUWorldPos;
    EXPECT_TRUE(testPosition.isZero());

    Position NEDWorldPos(1,2,3);
    Position NEDBodyPos = body->ProjectVectorInBody(NEDWorldPos,NED);
    testPosition = NEDWorldPos - NEDBodyPos;
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())){
        std::cout<<NEDWorldPos<<std::endl;
        std::cout<<NEDBodyPos<<std::endl;
    }

    testPosition = NWUBodyPos - body->ProjectVectorInWorld(NWUBodyPos,NWU);
    EXPECT_TRUE(testPosition.isZero());

    testPosition = NEDBodyPos - body->ProjectVectorInWorld(NEDBodyPos,NED);
    EXPECT_TRUE(testPosition.isZero());

}

TEST(FrBodyTest,ProjectVectorMethodsWithOrientation){

    FrOffshoreSystem system("system");

    // Body Instantiation
    auto body = system.NewBody("body");

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation TotalRotation = Rotation1*Rotation2 ;
    body->SetRotation(TotalRotation);

    const Position NWUWorldPos(1,2,3);

    Position NWUBodyPos = body->ProjectVectorInBody(NWUWorldPos,NWU);
    Position testPosition = NWUBodyPos - EasyRotateInv(NWUWorldPos,NWU);
    EXPECT_TRUE(testPosition.isZero());

    const Position NEDWorldPos(1,2,3);
    Position NEDBodyPos = body->ProjectVectorInBody(NEDWorldPos,NED);
    testPosition = NEDBodyPos - EasyRotateInv(NEDWorldPos,NED);
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())){
        std::cout<<NEDBodyPos<<std::endl;
        std::cout<<EasyRotateInv(NEDWorldPos,NED)<<std::endl;
    }

    testPosition = NWUWorldPos - body->ProjectVectorInWorld(NWUBodyPos,NWU);
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())){
        std::cout<<NWUWorldPos<<std::endl;
        std::cout<<NWUBodyPos<<std::endl;
    }

    testPosition = NEDWorldPos - body->ProjectVectorInWorld(NEDBodyPos,NED);
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())){
        std::cout<<NEDWorldPos<<std::endl;
        std::cout<<NEDBodyPos<<std::endl;
    }

}
