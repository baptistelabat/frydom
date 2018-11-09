//
// Created by Lucas Letournel on 05/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;


inline Position & EasyRotate(Position & vector) {
    Position res(vector.GetZ(),vector.GetX(),vector.GetY());
    return vector = res;
}

inline Translation & EasyRotate(Translation & vector) {
    Translation res(vector.GetDz(),vector.GetDx(),vector.GetDy());
    return vector = res;
}

inline Velocity & EasyRotate(Velocity & vector) {
    Velocity res(vector.GetVz(),vector.GetVx(),vector.GetVy());
    return vector = res;
}

void Test_AllGetPosition(const std::shared_ptr<FrBody_> body,
        const Position &AbsRefBodyPosition, const Position &AbsCOGBodyPosition,
        bool is_Orientation = false){

    Position ModAbsRefBodyPosition = AbsRefBodyPosition;
    Position ModAbsCOGBodyPosition = AbsCOGBodyPosition;
    if (is_Orientation) {
        ModAbsRefBodyPosition = EasyRotate(ModAbsRefBodyPosition);
        ModAbsCOGBodyPosition = EasyRotate(ModAbsCOGBodyPosition);
    }

    // Test body reference frame position in world reference frame
    Position testPosition = body->GetPosition(NWU) - AbsRefBodyPosition;
    EXPECT_TRUE(testPosition.isZero());

    //-----------------COG-----------------//
    // Test COG position in world reference frame
    testPosition = body->GetCOGPositionInWorld(NWU) - AbsCOGBodyPosition;
    EXPECT_TRUE(testPosition.isZero());
    // Test COG position in body reference frame
    Position TempPos = AbsCOGBodyPosition - AbsRefBodyPosition;
    if (is_Orientation) TempPos = EasyRotate(TempPos);
    testPosition = body->GetCOG(NWU) - TempPos;
//    std::cout<<body->GetCOGLocalPosition(NWU)<<std::endl;
//    std::cout<<TempPos<<std::endl;
    EXPECT_TRUE(testPosition.isZero());

    //-----------------Fixed Point-----------------//
    // Test for the getter for the local position of a point expressed in the world reference frame
    Position AbsPoint(4., 5., 6.); // Position of a point, expressed in world reference frame
    TempPos = AbsPoint - AbsRefBodyPosition;
    if (is_Orientation) TempPos = EasyRotate(TempPos);
    testPosition = body->GetPointPositionInBody(AbsPoint, NWU) - TempPos;
    EXPECT_TRUE(testPosition.isZero());

    Position BodyPoint(1.,5.,9.); // Position of a point, expressed in body reference frame
    // Test for the getter for the abs position of a point expressed in the body reference frame
    TempPos = body->GetPointPositionInWorld(BodyPoint, NWU) - AbsRefBodyPosition;
    if (is_Orientation) TempPos = EasyRotate(TempPos);
    testPosition = TempPos - BodyPoint;
    EXPECT_TRUE(testPosition.isZero());

}

TEST(FrBodyTest,TestPosition) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigAbsPos(1., 2., 3.);
    body->SetPosition(OrigAbsPos, NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2., 3., 4.);
    body->SetCOG(OrigLocalCOGPos, NWU);
    Position OrigAbsCOGPos = OrigAbsPos + OrigLocalCOGPos;

    Test_AllGetPosition(body,OrigAbsPos,OrigAbsCOGPos);
}

TEST(FrBodyTest,TestTranslation) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigAbsPos(1., 2., 3.);
    body->SetPosition(OrigAbsPos, NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2., 3., 4.);
    body->SetCOG(OrigLocalCOGPos, NWU);
    Position OrigAbsCOGPos = OrigAbsPos + OrigLocalCOGPos;

    //-----------------Translate Body-----------------//
    Translation BodyTranslationInWorld(1.,4.,7.);

    //+++++Translate body reference frame+++++//
    body->SetPosition(OrigAbsPos+BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,OrigAbsPos,OrigAbsCOGPos);

    //+++++Translate body reference frame from translation expressed in world reference frame+++++//
    body->TranslateInWorld(BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,OrigAbsPos + BodyTranslationInWorld,OrigAbsCOGPos + BodyTranslationInWorld);

    //+++++Translate body reference frame from translation expressed in body reference frame+++++//
    body->TranslateInBody(BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,OrigAbsPos + BodyTranslationInWorld,OrigAbsCOGPos + BodyTranslationInWorld);


    //+++++Translate body reference frame from COG+++++//
    body->SetPositionOfCOG(body->GetCOGAbsPosition(NWU) + BodyTranslationInWorld, NWU);
    Test_AllGetPosition(body,OrigAbsPos,OrigAbsCOGPos);

    //+++++Translate body reference frame from fixed point+++++//
    Position Point(4.,5.,6.); // Position of a point expressed in body reference frame
    body->SetPositionOfBodyPoint(Point, body->GetAbsPositionOfLocalPoint(Point, NWU), NWU);
    Test_AllGetPosition(body,OrigAbsPos,OrigAbsCOGPos);
}

TEST(FrBodyTest,TestOrientation) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigAbsPos(1., 2., 3.);
    body->SetPosition(OrigAbsPos, NWU);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    FrRotation_ OrigAbsRot;    OrigAbsRot.SetCardanAngles_DEGREES(1., 2., 3., NWU);
    body->SetRotation(OrigAbsRot);

    // Test of the getter for the absolute orientation (expressed in the world reference frame)
    EXPECT_TRUE(OrigAbsRot == body->GetRotation());

    // Test of the setter using cardan angles
    body->SetCardanAngles_DEGREES(1., 2., 3., NWU);
    EXPECT_TRUE(OrigAbsRot == body->GetRotation());
}

TEST(FrBodyTest,TestPositionWithOrientation){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigAbsPos(1.,2.,3.);
    body->SetPosition(OrigAbsPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOG(OrigLocalCOGPos, NWU);
    Position OrigAbsCOGPos = OrigAbsPos + OrigLocalCOGPos;

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2 ;
    body->SetRotation(TotalRotation);

//    std::cout<<body->GetAbsRotation();
//    std::cout<<body->GetAbsFrame();


    Test_AllGetPosition(body, OrigAbsPos, OrigAbsCOGPos, true);

    // Test that the orientation of the body changed the previous getter result
    Position OrigAbsPosPoint(4., 8., 5.); // Position of a point, expressed in world reference frame
    Position Test_LocalPosLocalPoint = (OrigAbsPosPoint - OrigAbsPos);
    auto EasyRotateTest = EasyRotate(Test_LocalPosLocalPoint);
    Position testPosition = body->GetPointPositionInBody(OrigAbsPosPoint,NWU)-EasyRotateTest;
    EXPECT_TRUE(testPosition.isZero());


}

void Test_AllGetVelocity(const std::shared_ptr<FrBody_> body,
                         const Velocity &VelocityToCompare,
                         bool is_Rotation) {

    Velocity RotVelocity = VelocityToCompare;
    if (is_Rotation) RotVelocity = EasyRotate(RotVelocity);


    // Test getter for the body velocity, expressed in the world reference frame
    Velocity testVelocity;
    testVelocity = body->GetVelocityInWorld(NWU);
    testVelocity -= VelocityToCompare;
    EXPECT_TRUE(testVelocity.isZero());

    // Test getter for the body velocity, expressed in the body reference frame
    testVelocity = body->GetVelocityInBody( NWU);
    testVelocity -= RotVelocity;
    EXPECT_TRUE(testVelocity.isZero());


    //-----------------COG Velocity-----------------//
    // Test Getter for the COG velocity expressed in the world reference frame
    testVelocity = body->GetCOGVelocityInWorld(NWU);
    testVelocity -= VelocityToCompare;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the COG velocity expressed in the body reference frame
    testVelocity = body->GetCOGVelocityInBody(NWU);
    testVelocity -= RotVelocity;
    EXPECT_TRUE(testVelocity.isZero());

    //-----------------Point-----------------//
    Position Point(5.,6.,7.);
    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in body reference frame
    body->GetVelocityInWorldAtPointInBody(testVelocity, NWU);
    testVelocity -= VelocityToCompare;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in world reference frame
    body->GetVelocityInWorldAtPointInWorld(testVelocity, NWU);
    testVelocity -= VelocityToCompare;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in body reference frame
    body->GetVelocityInBodyAtPointInBody(testVelocity, NWU);
    testVelocity -= RotVelocity;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in world reference frame
    body->GetVelocityInBodyAtPointInWorld(testVelocity, NWU);
    testVelocity -= RotVelocity;
    EXPECT_TRUE(testVelocity.isZero());
}

TEST(FrBodyTest,TestTranslationalVelocity){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOG(OrigLocalCOGPos, NWU);

    //-----------------Velocity Setters-----------------//
    //+++++Frame Velocity Setters+++++//
    // Set the body velocity, expressed in the world reference frame
    //Velocity VelocityInWorld(1.,1.,1.);    Velocity testVelocity;
    //body->SetVelocityInWorld(VelocityInWorld, NWU);
    //Test_AllGetVelocity(body,VelocityInWorld, false);

    // Set the body velocity, expressed in the body reference frame
    //body->SetVelocityInBody(VelocityInWorld,NWU);
    //Test_AllGetVelocity(body,VelocityInWorld, false);

    //+++++COG Velocity Setters+++++//
    // Test Setter for the COG Velocity expressed in the world reference frame
    Velocity COGVelocityInWorld(0.,1.,0.);
    body->SetCOGAbsVelocity(COGVelocityInWorld,NWU);
    Test_AllGetVelocity(body,COGVelocityInWorld, false);

    // Test Setter for the COG Velocity expressed in the body reference frame
    Velocity COGVelocityInBody(0.,1.,0.);
    body->SetCOGLocalVelocity(COGVelocityInBody,NWU);
    Test_AllGetVelocity(body,COGVelocityInBody, false);

//    //+++++Point Velocity Setters+++++//
//    Position PointInWorld(5.,6.,7.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,NWU);
//
//    // Test Setter for the generalized velocity expressed in the world reference frame,
//    // at a point expressed in world reference frame
//    Velocity VelocityInWorldAtPointInWorld(0.,1.,0.);
//    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,,NWU);
//
//    // Test Setter for the generalized velocity expressed in the world reference frame,
//    // at a point expressed in body reference frame
//    Velocity VelocityInWorldAtPointInBody(0.,1.,0.);
//    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,,NWU);
//
//    // Test Setter for the generalized velocity expressed in the body reference frame,
//    // at a point expressed in world reference frame
//    Velocity VelocityInBodyAtPointInWorld(0.,1.,0.);
//    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,,NWU);
//
//    // Test Setter for the generalized velocity expressed in the body reference frame,
//    // at a point expressed in body reference frame
//    Velocity VelocityInBodyAtPointInBody(0.,1.,0.);
//    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,,NWU);

}


TEST(FrBodyTest,TestTranslationalVelocityWithOrientation){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOG(OrigLocalCOGPos, NWU);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

    //-----------------Velocity Setters-----------------//
    //+++++Frame Velocity Setters+++++//
    // Set the body velocity, expressed in the world reference frame
    //Velocity VelocityInWorld(1.,1.,1.);    Velocity testVelocity;
    //body->SetVelocityInWorld(VelocityInWorld,NWU);
    //Test_AllGetVelocity(body, VelocityInWorld, true);

    // Set the body velocity, expressed in the body reference frame
    //body->SetVelocityInBody(VelocityInWorld,NWU);
    //Test_AllGetVelocity(body, VelocityInWorld, true);

    //+++++COG Velocity Setters+++++//
    // Test Setter for the COG Velocity expressed in the world reference frame
    Velocity COGVelocityInWorld(0.,1.,0.);
    body->SetCOGAbsVelocity(COGVelocityInWorld,NWU);
    Test_AllGetVelocity(body, COGVelocityInWorld, true);

    // Test Setter for the COG Velocity expressed in the body reference frame
    Velocity COGVelocityInBody(0.,1.,0.);
    body->SetCOGLocalVelocity(COGVelocityInBody,NWU);
    Test_AllGetVelocity(body, COGVelocityInBody, true);


}