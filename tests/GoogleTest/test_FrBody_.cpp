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

inline Velocity & EasyRotate(Velocity & vector) {
    Velocity res(vector.GetVz(),vector.GetVx(),vector.GetVy());
    return vector = res;
}

void Test_AllGetPosition(std::shared_ptr<FrBody_> body,
        const Position &AbsRefBodyPosition, const Position &AbsCOGBodyPosition,
        bool is_Orientation = false){
    Position ModAbsRefBodyPosition = AbsRefBodyPosition;
    Position ModAbsCOGBodyPosition = AbsCOGBodyPosition;
    if (is_Orientation) {
        ModAbsRefBodyPosition = EasyRotate(ModAbsRefBodyPosition);
        ModAbsCOGBodyPosition = EasyRotate(ModAbsCOGBodyPosition);
    }

    // Test body reference frame position in world reference frame
    Position testPosition = body->GetAbsPosition(NWU) - AbsRefBodyPosition;
    EXPECT_TRUE(testPosition.isZero());

    //-----------------COG-----------------//
    // Test COG position in world reference frame
    testPosition = body->GetCOGAbsPosition(NWU) - AbsCOGBodyPosition;
    EXPECT_TRUE(testPosition.isZero());
    // Test COG position in body reference frame
    Position TempPos = AbsCOGBodyPosition - AbsRefBodyPosition;
    if (is_Orientation) TempPos = EasyRotate(TempPos);
    testPosition = body->GetCOGLocalPosition(NWU) - TempPos;
//    std::cout<<body->GetCOGLocalPosition(NWU)<<std::endl;
//    std::cout<<TempPos<<std::endl;
    EXPECT_TRUE(testPosition.isZero());

    //-----------------Fixed Point-----------------//
    // Test for the getter for the local position of a point expressed in the world reference frame
    Position AbsPoint(4., 5., 6.); // Position of a point, expressed in world reference frame
    TempPos = AbsPoint - AbsRefBodyPosition;
    if (is_Orientation) TempPos = EasyRotate(TempPos);
    testPosition = body->GetLocalPositionOfAbsPoint(AbsPoint, NWU) - TempPos;
    EXPECT_TRUE(testPosition.isZero());

    Position BodyPoint(1.,5.,9.); // Position of a point, expressed in body reference frame
    // Test for the getter for the abs position of a point expressed in the body reference frame
    TempPos = body->GetAbsPositionOfLocalPoint(BodyPoint, NWU) - AbsRefBodyPosition;
    if (is_Orientation) TempPos = EasyRotate(TempPos);
    testPosition = TempPos - BodyPoint;
    EXPECT_TRUE(testPosition.isZero());

}

TEST(FrBodyTest,TestPosition) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigAbsPos(1., 2., 3.);
    body->SetAbsPosition(OrigAbsPos, NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2., 3., 4.);
    body->SetCOGLocalPosition(OrigLocalCOGPos, false, NWU);
    Position OrigAbsCOGPos = OrigAbsPos + OrigLocalCOGPos;

    Test_AllGetPosition(body,OrigAbsPos,OrigAbsCOGPos);
}

//TEST(FrBodyTest,TestTranslation) {
//    // Body Instantiation
//    auto body = std::make_shared<FrBody_>();
//
//    Position OrigAbsPos(1., 2., 3.);
//    body->SetAbsPosition(OrigAbsPos, NWU);
//
//    //-----------------COG-----------------//
//    // Set the COG position, expressed in local body reference frame
//    Position OrigLocalCOGPos(2., 3., 4.);
//    body->SetCOGLocalPosition(OrigLocalCOGPos, false, NWU);
//    Position OrigAbsCOGPos = OrigAbsPos + OrigLocalCOGPos;
//
//    //-----------------Translate Body-----------------//
//    Translation BodyTranslation(1.,4.,7.);
////    body->Translate(BodyTranslation,NWU);
//    Test_AllGetPosition(body,OrigAbsPos,OrigAbsCOGPos);
//
//    //+++++Translate body reference frame+++++//
////    body->SetPosition(OrigAbsPos+BodyTranslation,NWU);
//    Test_AllGetPosition(body,OrigAbsPos,OrigAbsCOGPos);
//
//    //+++++Translate body reference frame from COG+++++//
////    body->SetCOGPosition(body->GetCOGAbsPosition(NWU)+BodyTranslation,NWU);
//    Test_AllGetPosition(body,OrigAbsPos,OrigAbsCOGPos);
//
//    //+++++Translate body reference frame from fixed point+++++//
//    Position Point(4.,5.,6.); // Position of a point expressed in body reference frame
////    body-SetPointPosition(Point, body->GetAbsPositionOfLocalPoint(Point,NWU), NWU);
//    Test_AllGetPosition(body,OrigAbsPos,OrigAbsCOGPos);
//}

TEST(FrBodyTest,TestOrientation) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigAbsPos(1., 2., 3.);
    body->SetAbsPosition(OrigAbsPos, NWU);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    FrRotation_ OrigAbsRot;    OrigAbsRot.SetCardanAngles_DEGREES(1., 2., 3., NWU);
    body->SetAbsRotation(OrigAbsRot);

    // Test of the getter for the absolute orientation (expressed in the world reference frame)
    EXPECT_TRUE(OrigAbsRot == body->GetAbsRotation());

    // Test of the setter using cardan angles
    body->SetCardanAngles_DEGREES(1., 2., 3., NWU);
    EXPECT_TRUE(OrigAbsRot == body->GetAbsRotation());
}

TEST(FrBodyTest,TestPositionWithOrientation){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigAbsPos(1.,2.,3.);
    body->SetAbsPosition(OrigAbsPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOGLocalPosition(OrigLocalCOGPos, false, NWU);
    Position OrigAbsCOGPos = OrigAbsPos + OrigLocalCOGPos;

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2 ;
    body->SetAbsRotation(TotalRotation);

//    std::cout<<body->GetAbsRotation();
//    std::cout<<body->GetAbsFrame();


    Test_AllGetPosition(body, OrigAbsPos, OrigAbsCOGPos, true);

    // Test that the orientation of the body changed the previous getter result
    Position OrigAbsPosPoint(4., 8., 5.); // Position of a point, expressed in world reference frame
    Position Test_LocalPosLocalPoint = (OrigAbsPosPoint - OrigAbsPos);
//    Position testPosition = body->GetLocalPositionOfAbsPoint(OrigAbsPosPoint,NWU)-Test_LocalPosLocalPoint;
//    std::cout<< body->GetLocalPositionOfAbsPoint(OrigAbsPosPoint,NWU) << std::endl;
//    std::cout<< Test_LocalPosLocalPoint << std::endl;
//    EXPECT_TRUE(testPosition.isZero());

    auto EasyRotateTest = EasyRotate(Test_LocalPosLocalPoint);
    Position testPosition = body->GetLocalPositionOfAbsPoint(OrigAbsPosPoint,NWU)-EasyRotateTest;
//    std::cout<< body->GetLocalPositionOfAbsPoint(OrigAbsPosPoint,NWU) << std::endl;
//    std::cout<< EasyRotateTest << std::endl;
    EXPECT_TRUE(testPosition.isZero());


}

TEST(FrBodyTest,TestTranslationalVelocity){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigWorldPos(1.,2.,3.);
    body->SetAbsPosition(OrigWorldPos,NWU);

    //-----------------Frame Velocity-----------------//
    Velocity VelocityInWorld(1.,1.,1.);
    body->SetAbsVelocity(VelocityInWorld,NWU);
    Velocity testVelocity = body->GetAbsVelocity(NWU) - VelocityInWorld;
//    std::cout<<testVelocity;
    EXPECT_TRUE(testVelocity.isZero());
    testVelocity = body->GetLocalVelocity(NWU) - VelocityInWorld;
//    std::cout<<testVelocity;
    EXPECT_TRUE(testVelocity.isZero());

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOGLocalPosition(OrigLocalCOGPos, false, NWU);

    //-----------------COG Velocity-----------------//
    // Test Getter for the COG velocity expressed in the World reference frame
    testVelocity = body->GetCOGAbsVelocity(NWU) - VelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());
    // Test Getter for the COG velocity expressed in the Body reference frame
    testVelocity = body->GetCOGLocalVelocity(NWU) - VelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Setter for the COG Velocity expressed in the World reference frame
    Velocity COGVelocityInWorld(0.,1.,0.);
    body->SetCOGAbsVelocity(COGVelocityInWorld,NWU);
    testVelocity = body->GetCOGAbsVelocity(NWU) - COGVelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());
    // Test Setter for the COG Velocity expressed in the Body reference frame
    Velocity COGVelocityInBody(0.,1.,0.);
    body->SetCOGLocalVelocity(COGVelocityInBody,NWU);
    testVelocity = body->GetCOGAbsVelocity(NWU) - COGVelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());

    //-----------------Point-----------------//
    Position Point(5.,6.,7.);
    // Test Getter for the velocity expressed in the World reference frame, at a Point expressed in Body reference frame
    testVelocity = body->GetAbsVelocityOfLocalPoint(Point,NWU)- COGVelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Setter for the COG Velocity expressed in the World reference frame
    Velocity PointVelocityInWorld(0.,1.,0.);
//    body->SetVelocityInWorldAtPointInBody(Point,PointVelocityInWorld,NWU);
    testVelocity = body->GetAbsVelocityOfLocalPoint(Point,NWU) - PointVelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());
    // Test Setter for the COG Velocity expressed in the Body reference frame
    Velocity PointVelocityInBody(0.,1.,0.);
//    body->SetVelocityInBodyAtPointInBody(Point,PointVelocityInBody,NWU);
//    testVelocity = body->GetLocalVelocityOfLocalPoint(Point,NWU) - PointVelocityInBody;
//    EXPECT_TRUE(testVelocity.isZero());

}


TEST(FrBodyTest,TestTranslationalVelocityWithOrientation){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigWorldPos(1.,2.,3.);
    body->SetAbsPosition(OrigWorldPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOGLocalPosition(OrigLocalCOGPos, false, NWU);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2;
    body->SetAbsRotation(TotalRotation);

    //-----------------Frame Velocity-----------------//
    Velocity VelocityInWorld(1.,1.,1.);
    body->SetAbsVelocity(VelocityInWorld,NWU);

    // Velocity in the Body reference frame
    Velocity VelocityInBodyFromRotate = EasyRotate(VelocityInWorld);

    // Test of the Velocity getter in the World reference frame
    Velocity testVelocity = body->GetAbsVelocity(NWU) - VelocityInWorld;
//    std::cout<<testVelocity;
    EXPECT_TRUE(testVelocity.isZero());

    // Test of the Velocity getter in the Body reference frame
    testVelocity = body->GetLocalVelocity(NWU) - VelocityInBodyFromRotate;
//    std::cout<<testVelocity;
    EXPECT_TRUE(testVelocity.isZero());

    //-----------------COG Velocity-----------------//

    // Test Getter for the COG velocity expressed in the Body reference frame
    testVelocity = body->GetCOGAbsVelocity(NWU) - VelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the COG velocity expressed in the Body reference frame
    testVelocity = body->GetCOGLocalVelocity(NWU) - VelocityInBodyFromRotate;
    EXPECT_TRUE(testVelocity.isZero());

    //-----------------Point Velocity-----------------//
    Position Point(5.,6.,7.);

    // Test Getter for the Point Velocity expressed in the Body reference frame
    testVelocity = body->GetAbsVelocityOfLocalPoint(Point,NWU) - VelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the Point Velocity expressed in the Body reference frame
//    testVelocity = body->GetLocalVelocityOfLocalPoint(Point,NWU) - VelocityInBodyFromRotate;
//    EXPECT_TRUE(testVelocity.isZero());

}