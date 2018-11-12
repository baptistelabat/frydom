//
// Created by Lucas Letournel on 05/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

template <class Vector>
inline Vector& EasyRotate(Vector& vector) {
    Vector vecTem = vector;
    vecTem[0] = vector[2];
    vecTem[1] = vector[0];
    vecTem[2] = vector[1];
    return vector = vecTem;
}

template <class Vector>
inline Vector EasyRotate(const Vector& vector) {
    Vector out = vector;
    return EasyRotate<Vector>(out);
}

template <class Vector>
inline Vector& EasyRotateInv(Vector& vector) {
    Vector vecTem = vector;
    vecTem[0] = vector[1];
    vecTem[1] = vector[2];
    vecTem[2] = vector[0];
    return vector = vecTem;
}

template <class Vector>
inline Vector EasyRotateInv(const Vector& vector) {
    Vector out = vector;
    return EasyRotate<Vector>(out);
}


//inline Position & EasyRotate(Position & vector) {
//    Position res(vector.GetZ(),vector.GetX(),vector.GetY());
////    Position res(vector.GetX(),-vector.GetZ(),vector.GetY());
//    return vector = res;
//}
//
//inline Translation & EasyRotate(Translation & vector) {
//    Translation res(vector.GetDz(),vector.GetDx(),vector.GetDy());
////    Translation res(vector.GetDx(),-vector.GetDz(),vector.GetDy());
//    return vector = res;
//}
//
//inline Velocity & EasyRotate(Velocity & vector) {
//    Velocity res(vector.GetVz(),vector.GetVx(),vector.GetVy());
////    Velocity res(vector.GetVx(),-vector.GetVz(),vector.GetVy());
//    return vector = res;
//}
//
//inline AngularVelocity & EasyRotate(AngularVelocity & vector) {
//    Velocity res(vector.GetWz(),vector.GetWx(),vector.GetWy());
////    Velocity res(vector.GetVx(),-vector.GetVz(),vector.GetVy());
//    return vector = res;
//}
//
//
//inline Velocity & EasyRotateInv(Velocity & vector) {
//    Velocity res(vector.GetVy(),vector.GetVz(),vector.GetVx());
//    return vector = res;
//}


void Test_AllGetPosition(const std::shared_ptr<FrBody_> body,
        const Position &RefPositionInWorld, const Position &COGPositionInWorld,
        bool is_Orientation = false){

    // Test body reference frame position in world reference frame
    Position testPosition = body->GetPosition(NWU) - RefPositionInWorld;
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())) {
        std::cout<<body->GetPosition(NWU)<<std::endl;
        std::cout<<RefPositionInWorld<<std::endl;
    }

    //-----------------COG-----------------//
    // Test COG position in body reference frame
    Position TempPos = COGPositionInWorld - RefPositionInWorld;
    testPosition = body->GetCOG(NWU) - TempPos;
    EXPECT_TRUE(testPosition.isZero());
    // Test COG position in world reference frame
    TempPos = COGPositionInWorld - RefPositionInWorld;
    if (is_Orientation) EasyRotate(TempPos);
    testPosition = body->GetCOGPositionInWorld(NWU) - (RefPositionInWorld +TempPos);
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())) {
        std::cout<<body->GetCOGPositionInWorld(NWU)<<std::endl;
        std::cout<<COGPositionInWorld<<std::endl;
    }

    //-----------------Fixed Point-----------------//
    // Test for the getter for the local position of a point expressed in the world reference frame
    Position PointPositionInWorld(1., 5., 9.); // Position of a point, expressed in world reference frame
    TempPos = body->GetPointPositionInBody(PointPositionInWorld, NWU);
    if (is_Orientation) EasyRotate(TempPos);
    testPosition = TempPos - (PointPositionInWorld - RefPositionInWorld);
    EXPECT_TRUE(testPosition.isZero());

    Position PointPositionInBody(1.,5.,9.); // Position of a point, expressed in body reference frame
    // Test for the getter for the abs position of a point expressed in the body reference frame
    TempPos = PointPositionInBody;
    if (is_Orientation) EasyRotate(TempPos);
    testPosition = body->GetPointPositionInWorld(PointPositionInBody, NWU) - RefPositionInWorld - TempPos;
    EXPECT_TRUE(testPosition.isZero());

}

TEST(FrBodyTest,Position) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2., 3., 4.);
    body->SetCOG(COGPositionInBody, NWU);
    Position COGPositionInWorld = RefPositionInWorld + COGPositionInBody;

    Test_AllGetPosition(body,RefPositionInWorld,COGPositionInWorld);
}

TEST(FrBodyTest,Translation) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2., 3., 4.);
    body->SetCOG(COGPositionInBody, NWU);
    Position COGPositionInWorld = RefPositionInWorld + COGPositionInBody;

    //-----------------Translate Body-----------------//
    Translation BodyTranslationInWorld(1.,4.,7.);
    Position BodyPosition = RefPositionInWorld + BodyTranslationInWorld;
    Position COGPosition = COGPositionInWorld + BodyTranslationInWorld;

    //+++++Translate body reference frame+++++//
//    std::cout<<"+++++SetPosition"<<std::endl;
    body->SetPosition(RefPositionInWorld+BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition);

    //+++++Translate body reference frame from fixed point+++++//
//    std::cout<<"+++++SetPointPosition"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    Position Point(4.,5.,6.); // Position of a point expressed in body reference frame
    body->SetPositionOfBodyPoint(Point, body->GetPointPositionInWorld(Point,NWU) + BodyTranslationInWorld, NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition);

    //+++++Translate body reference frame from translation expressed in world reference frame+++++//
//    std::cout<<"+++++TranslateInWorld"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInWorld(BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition);

    //+++++Translate body reference frame from translation expressed in body reference frame+++++//
//    std::cout<<"+++++TranslateInBody"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInBody(BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition);

}


void Test_AllGetRotation(const std::shared_ptr<FrBody_> body, const FrRotation_ &BodyRotationInWorld){

    // Test of the getter for the absolute orientation (expressed in the world reference frame)
    EXPECT_TRUE(body->GetRotation() == BodyRotationInWorld);

    // Test of the getter for the quaternion
    EXPECT_TRUE(body->GetQuaternion() == BodyRotationInWorld.GetQuaternion());

    // Test of the frame getter
    EXPECT_TRUE(body->GetFrame().GetRotation() == BodyRotationInWorld);
}

TEST(FrBodyTest,Orientation) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, NWU);

    //-----------------Orientation-----------------//
    //+++++Set Rotation+++++//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    FrRotation_ BodyRotationInWorld;    BodyRotationInWorld.SetCardanAngles_DEGREES(1., 2., 3., NWU);
    body->SetRotation(BodyRotationInWorld);

    Test_AllGetRotation(body, BodyRotationInWorld);

//    //+++++Set Rotation, using Cardan angles+++++//
//    body->SetCardanAngles_DEGREES(1., 2., 3., NWU);
//    EXPECT_TRUE(BodyRotationInWorld == body->GetRotation());

    Test_AllGetRotation(body, BodyRotationInWorld);
    // Test of the frame position getter
    Position testPosition = body->GetFrame().GetPosition(NWU) - RefPositionInWorld;
    EXPECT_TRUE(testPosition.isZero());

    //+++++Set Frame+++++//
    FrFrame_ RefFrame(RefPositionInWorld,BodyRotationInWorld,NWU);
    body->SetFrame(RefFrame);

    Test_AllGetRotation(body, BodyRotationInWorld);
    // Test of the frame position getter
    testPosition = body->GetFrame().GetPosition(NWU) - RefFrame.GetPosition(NWU);
    EXPECT_TRUE(testPosition.isZero());

}

TEST(FrBodyTest,PositionWithOrientation){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position RefPositionInWorld(1.,2.,3.);
    body->SetPosition(RefPositionInWorld,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2.,3.,4.);
    body->SetCOG(COGPositionInBody, NWU);
    Position OrigAbsCOGPos = RefPositionInWorld + COGPositionInBody;

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2 ;
    body->SetRotation(TotalRotation);


    Test_AllGetPosition(body, RefPositionInWorld, OrigAbsCOGPos, true);

}

void Test_GetAngularVelocity(const std::shared_ptr<FrBody_> body,
                             const AngularVelocity &BodyAngularVelocityInWorld, bool is_orientation){

    AngularVelocity testAngularVelocity;

    // Test Angular Velocity getter, expressed in world reference frame
    testAngularVelocity = body->GetAngularVelocityInWorld(NWU) - BodyAngularVelocityInWorld;
    EXPECT_TRUE(testAngularVelocity.isZero());

    // Test Angular Velocity getter, expressed in body reference frame
    testAngularVelocity = body->GetAngularVelocityInBody(NWU);
    if (is_orientation) EasyRotate(testAngularVelocity);
    testAngularVelocity -= BodyAngularVelocityInWorld;
    EXPECT_TRUE(testAngularVelocity.isZero());
}

TEST(FrBodyTest,AngularVelocity) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    AngularVelocity BodyAngularVelocityInBody, BodyAngularVelocityInWorld;

    //+++++Test Angular Velocity Setters and Getters without any body reference frame rotation+++++//
    // Test Angular Velocity setter, expressed in world reference frame
    BodyAngularVelocityInWorld.Set(1.,2.,3.);
    body->SetAngularVelocityInWorld(BodyAngularVelocityInWorld,NWU);
//    std::cout<<"SetAngularVelocityInWorld"<<std::endl;
    Test_GetAngularVelocity(body,BodyAngularVelocityInWorld,false);


    // Test Angular Velocity setter, expressed in body reference frame
    BodyAngularVelocityInBody.Set(4.,5.,6.);
    body->SetAngularVelocityInBody(BodyAngularVelocityInBody,NWU);
//    std::cout<<"SetAngularVelocityInBody"<<std::endl;
    Test_GetAngularVelocity(body,BodyAngularVelocityInBody,false);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

    //+++++Test Angular Velocity Setters and Getters with a body reference frame rotation+++++//
    // Test Angular Velocity setter, expressed in world reference frame
    BodyAngularVelocityInWorld.Set(1.,2.,3.);
    body->SetAngularVelocityInWorld(BodyAngularVelocityInWorld,NWU);
//    std::cout<<"SetAngularVelocityInWorld, with a rotation"<<std::endl;
    Test_GetAngularVelocity(body,BodyAngularVelocityInWorld,true);


    // Test Angular Velocity setter, expressed in body reference frame
    BodyAngularVelocityInBody.Set(4.,5.,6.);
    body->SetAngularVelocityInBody(BodyAngularVelocityInBody,NWU);
//    std::cout<<"SetAngularVelocityInBody, with a rotation"<<std::endl;
    BodyAngularVelocityInWorld = EasyRotate(BodyAngularVelocityInBody);
    Test_GetAngularVelocity(body,BodyAngularVelocityInWorld,true);


}


void Test_AllGetVelocity(const std::shared_ptr<FrBody_> body,
                         const Velocity &VelocityInWorld,
                         bool is_Rotation) {

    Velocity VelocityInBody = VelocityInWorld;
    if (is_Rotation) EasyRotateInv(VelocityInBody);


    // Test getter for the body velocity, expressed in the world reference frame
    Velocity testVelocity;
    testVelocity = body->GetVelocityInWorld(NWU) - VelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())) {
        std::cout<<VelocityInWorld<<std::endl;
        std::cout<<body->GetVelocityInWorld(NWU)<<std::endl;
    }
    // Test getter for the body velocity, expressed in the body reference frame
    testVelocity = body->GetVelocityInBody(NWU) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());
//    std::cout<<body->GetVelocityInBody(NWU)<<std::endl;


    //-----------------COG Velocity-----------------//
    // Test Getter for the COG velocity expressed in the world reference frame
    testVelocity = body->GetCOGVelocityInWorld(NWU) - VelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the COG velocity expressed in the body reference frame
    testVelocity = body->GetCOGVelocityInBody(NWU) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());


    //-----------------PointInBody-----------------//
    Position PointInBody(5.,6.,7.);    Position PointInWorld(7.,6.,5.);
    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in body reference frame
    testVelocity = body->GetVelocityInWorldAtPointInBody(PointInBody, NWU) - VelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in world reference frame
    testVelocity = body->GetVelocityInWorldAtPointInWorld(PointInWorld, NWU) - VelocityInWorld;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in body reference frame
    testVelocity = body->GetVelocityInBodyAtPointInBody(PointInBody, NWU) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())) {
        std::cout<<VelocityInBody<<std::endl;
        std::cout<<body->GetVelocityInBody(NWU)<<std::endl;
        std::cout<<body->GetVelocityInBodyAtPointInBody(PointInBody, NWU)<<std::endl;
    }

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in world reference frame
    testVelocity = body->GetVelocityInBodyAtPointInWorld(PointInWorld, NWU) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());
}

TEST(FrBodyTest,TranslationalVelocity){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOG(OrigLocalCOGPos, NWU);

    //-----------------Angular Velocity-----------------//
    AngularVelocity BodyAngularVelocity(0,0,0);
    bool is_orientation = false;

    //-----------------Velocity Setters-----------------//
    //+++++COG Frame Velocity Setters+++++//
    // Set the body velocity, expressed in the world reference frame
    Velocity VelocityInWorld(1.,1.,1.);
    body->SetVelocityInWorldNoRotation(VelocityInWorld,NWU);
    body->UpdateRefFrame(true);
    Test_AllGetVelocity(body,VelocityInWorld, is_orientation);

    // Set the body velocity, expressed in the body reference frame
    Velocity VelocityInBody(1.,6.,4.);
    body->SetVelocityInBodyNoRotation(VelocityInBody,NWU);
    body->UpdateRefFrame(true);
    Test_AllGetVelocity(body,VelocityInBody, is_orientation);

    // Set the body generalized velocity, expressed in the world reference frame
    body->SetGeneralizedVelocityInWorld(VelocityInWorld, BodyAngularVelocity, NWU);
    body->UpdateRefFrame(true);
    Test_AllGetVelocity(body,VelocityInWorld, is_orientation);

    // Set the body generalized velocity, expressed in the body reference frame
    body->SetGeneralizedVelocityInBody(VelocityInBody, BodyAngularVelocity, NWU);
    body->UpdateRefFrame(true);
    Test_AllGetVelocity(body,VelocityInBody, is_orientation);

    //+++++Point Velocity Setters+++++//
    Position PointInWorld(5.,6.,7.); Position PointInBody(7.,6.,5.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,NWU);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInWorld"<<std::endl;
    Velocity VelocityInWorldAtPointInWorld(3.,1.,5.);
    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,BodyAngularVelocity,NWU);
    body->UpdateRefFrame(true);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInWorld, is_orientation);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInBody"<<std::endl;
    Velocity VelocityInWorldAtPointInBody(2.,1.,8.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,BodyAngularVelocity,NWU);
    body->UpdateRefFrame(true);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInBody, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInWorld"<<std::endl;
    Velocity VelocityInBodyAtPointInWorld(8.,1.,9.);
    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,BodyAngularVelocity,NWU);
    body->UpdateRefFrame(true);

    Test_AllGetVelocity(body, VelocityInBodyAtPointInWorld, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInBody"<<std::endl;
    Velocity VelocityInBodyAtPointInBody(1.,1.,5.);
    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,BodyAngularVelocity,NWU);
    body->UpdateRefFrame(true);

    Test_AllGetVelocity(body, VelocityInBodyAtPointInBody, is_orientation);

}


TEST(FrBodyTest,TranslationalVelocityWithOrientation){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,NWU);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

    bool is_orientation = true;

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOG(OrigLocalCOGPos, NWU);

    //-----------------Angular Velocity-----------------//
    AngularVelocity BodyAngularVelocity(0,0,0);

    //-----------------Velocity Setters-----------------//
    //+++++COG Frame Velocity Setters+++++//
    // Set the body velocity, expressed in the world reference frame
    Velocity VelocityInWorld(1.,1.,1.);
    std::cout<<"SetVelocityInWorldNoRotation"<<std::endl;
    body->SetVelocityInWorldNoRotation(VelocityInWorld,NWU);
    body->UpdateRefFrame(true);
    Test_AllGetVelocity(body,VelocityInWorld, is_orientation);

    // Set the body velocity, expressed in the body reference frame
    const Velocity VelocityInBody(1.,6.,4.);
    VelocityInWorld = EasyRotate(VelocityInBody);
    std::cout<<"SetVelocityInBodyNoRotation"<<std::endl;
    body->SetVelocityInBodyNoRotation(VelocityInBody,NWU);
    body->UpdateRefFrame(true);
    Test_AllGetVelocity(body,VelocityInWorld, is_orientation);

    // Set the body generalized velocity, expressed in the world reference frame
    std::cout<<"SetGeneralizedVelocityInWorld"<<std::endl;
    body->SetGeneralizedVelocityInWorld(VelocityInWorld, BodyAngularVelocity, NWU);
    body->UpdateRefFrame(true);
    Test_AllGetVelocity(body,VelocityInWorld, is_orientation);

    // Set the body generalized velocity, expressed in the body reference frame
    std::cout<<"SetGeneralizedVelocityInBody"<<std::endl;
    body->SetGeneralizedVelocityInBody(VelocityInBody, BodyAngularVelocity, NWU);
    body->UpdateRefFrame(true);
    Test_AllGetVelocity(body,VelocityInWorld, is_orientation);

    //+++++Point Velocity Setters+++++//
    Position PointInWorld(5.,6.,7.); Position PointInBody(7.,6.,5.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,NWU);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in world reference frame
    std::cout<<"SetGeneralizedVelocityInWorldAtPointInWorld"<<std::endl;
    Velocity VelocityInWorldAtPointInWorld(3.,1.,5.);
    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,BodyAngularVelocity,NWU);
    body->UpdateRefFrame(true);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInWorld, is_orientation);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in body reference frame
    std::cout<<"SetGeneralizedVelocityInWorldAtPointInBody"<<std::endl;
    Velocity VelocityInWorldAtPointInBody(2.,1.,8.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,BodyAngularVelocity,NWU);
    body->UpdateRefFrame(true);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInBody, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in world reference frame
    std::cout<<"SetGeneralizedVelocityInBodyAtPointInWorld"<<std::endl;
    const Velocity VelocityInBodyAtPointInWorld(8.,1.,9.);
    VelocityInWorldAtPointInWorld = EasyRotate(VelocityInBodyAtPointInWorld);
    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,BodyAngularVelocity,NWU);
    body->UpdateRefFrame(true);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInWorld, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in body reference frame
    const Velocity VelocityInBodyAtPointInBody(1.,1.,5.);
    VelocityInWorldAtPointInBody = EasyRotate(VelocityInBodyAtPointInBody);
    std::cout<<"SetGeneralizedVelocityInBodyAtPointInBody"<<std::endl;
    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,BodyAngularVelocity,NWU);
    body->UpdateRefFrame(true);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInBody, is_orientation);
}

