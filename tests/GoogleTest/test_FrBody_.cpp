//
// Created by Lucas Letournel on 05/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

// From Body To World
template <class Vector>
inline Vector& EasyRotate(Vector& vector, FRAME_CONVENTION fc=NWU) {
    Vector vecTem = vector;
    if (fc==NED) {internal::SwapFrameConvention(vector);}
    vecTem[0] = vector[2];
    vecTem[1] = vector[0];
    vecTem[2] = vector[1];
    if (fc==NED) {internal::SwapFrameConvention(vecTem);}
    return vector = vecTem;
}

template <class Vector>
inline Vector EasyRotate(const Vector& vector, FRAME_CONVENTION fc=NWU) {
    Vector out = vector;
    return EasyRotate<Vector>(out,fc);
}

// From World To Body
template <class Vector>
inline Vector& EasyRotateInv(Vector& vector, FRAME_CONVENTION fc=NWU) {
    Vector vecTem = vector;
    if (fc==NED) {internal::SwapFrameConvention(vector);}
    vecTem[0] = vector[1];
    vecTem[1] = vector[2];
    vecTem[2] = vector[0];
    if (fc==NED) {internal::SwapFrameConvention(vecTem);}
    return vector = vecTem;
}

template <class Vector>
inline Vector EasyRotateInv(const Vector& vector, FRAME_CONVENTION fc=NWU) {
    Vector out = vector;
    return EasyRotateInv<Vector>(out,fc);
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
        Position &RefPositionInWorld, Position &COGPositionInWorld, FRAME_CONVENTION in_fc,
        bool is_Orientation, FRAME_CONVENTION out_fc){

    if (out_fc != in_fc) {
        internal::SwapFrameConvention(RefPositionInWorld);
        internal::SwapFrameConvention(COGPositionInWorld);
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
    if (is_Orientation) EasyRotateInv(TempPos);
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
    if (is_Orientation) EasyRotate(TempPos);
    testPosition = TempPos - (PointPositionInWorld - RefPositionInWorld);
    EXPECT_TRUE(testPosition.isZero());

    Position PointPositionInBody(1.,5.,9.); // Position of a point, expressed in body reference frame
    // Test for the getter for the abs position of a point expressed in the body reference frame
    TempPos = PointPositionInBody;
    if (is_Orientation) EasyRotate(TempPos);
    testPosition = body->GetPointPositionInWorld(PointPositionInBody, out_fc) - RefPositionInWorld - TempPos;
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

    Test_AllGetPosition(body,RefPositionInWorld,COGPositionInWorld,NWU,false,NWU);
}

TEST(FrBodyTest,PositionNED) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2., 3., 4.);
    body->SetCOG(COGPositionInBody, NWU);
    Position COGPositionInWorld = RefPositionInWorld + COGPositionInBody;

    Test_AllGetPosition(body,RefPositionInWorld,COGPositionInWorld,NWU,false,NED);
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
    Test_AllGetPosition(body,BodyPosition,COGPosition,NWU,false,NWU);

    //+++++Translate body reference frame from fixed point+++++//
//    std::cout<<"+++++SetPointPosition"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    Position Point(4.,5.,6.); // Position of a point expressed in body reference frame
    body->SetPositionOfBodyPoint(Point, body->GetPointPositionInWorld(Point,NWU) + BodyTranslationInWorld, NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition,NWU,false,NWU);

    //+++++Translate body reference frame from translation expressed in world reference frame+++++//
//    std::cout<<"+++++TranslateInWorld"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInWorld(BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition,NWU,false,NWU);

    //+++++Translate body reference frame from translation expressed in body reference frame+++++//
//    std::cout<<"+++++TranslateInBody"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInBody(BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition,NWU,false,NWU);

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2 ;
    body->SetRotation(TotalRotation);

    //+++++Translate body reference frame from fixed point, with a rotation+++++//
    COGPosition = body->GetCOGPositionInWorld(NWU);
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInBody(EasyRotateInv(BodyTranslationInWorld),NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition,NWU,true,NWU);

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

//    Test_AllGetRotation(body, BodyRotationInWorld);

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

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2 ;
    body->SetRotation(TotalRotation);

    Position OrigAbsCOGPos = RefPositionInWorld + EasyRotate(COGPositionInBody);

    Test_AllGetPosition(body, RefPositionInWorld, OrigAbsCOGPos, NWU, true, NWU);

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
                         const Velocity &VelocityInWorld, const AngularVelocity &AngularVelocityInWorld,
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
    testVelocity -= AngularVelocityInWorld.cross(body->GetCOGPositionInWorld(NWU) - body->GetPosition(NWU));
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())) {
        std::cout<<VelocityInWorld +
        AngularVelocityInWorld.cross(body->GetCOGPositionInWorld(NWU) - body->GetPosition(NWU))<<std::endl;
        std::cout<<body->GetCOGVelocityInWorld(NWU)<<std::endl;
    }

    // Test Getter for the COG velocity expressed in the body reference frame
    testVelocity = - AngularVelocityInWorld.cross(body->GetCOGPositionInWorld(NWU) - body->GetPosition(NWU));
    if (is_Rotation) testVelocity = EasyRotateInv(testVelocity);
    testVelocity += body->GetCOGVelocityInBody(NWU) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());


    //-----------------PointInBody-----------------//
    Position PointInBody(5.,6.,7.);    Position PointInWorld(7.,6.,5.);
    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in body reference frame
    testVelocity = body->GetVelocityInWorldAtPointInBody(PointInBody, NWU) - VelocityInWorld;
    testVelocity -= AngularVelocityInWorld.cross(body->GetPointPositionInWorld(PointInBody, NWU) - body->GetPosition(NWU));
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in world reference frame
    testVelocity = body->GetVelocityInWorldAtPointInWorld(PointInWorld, NWU) - VelocityInWorld;
    testVelocity -= AngularVelocityInWorld.cross(PointInWorld - body->GetPosition(NWU));
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in body reference frame
    testVelocity = - AngularVelocityInWorld.cross(body->GetPointPositionInWorld(PointInBody, NWU) - body->GetPosition(NWU));
    if (is_Rotation) testVelocity = EasyRotateInv(testVelocity);
    testVelocity += body->GetVelocityInBodyAtPointInBody(PointInBody, NWU) - VelocityInBody;
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())) {
        std::cout<<VelocityInBody<<std::endl;
        std::cout<<body->GetVelocityInBody(NWU)<<std::endl;
        std::cout<<body->GetVelocityInBodyAtPointInBody(PointInBody, NWU)<<std::endl;
    }

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in world reference frame
    testVelocity = - AngularVelocityInWorld.cross(PointInWorld - body->GetPosition(NWU));
    if (is_Rotation) testVelocity = EasyRotateInv(testVelocity);
    testVelocity += body->GetVelocityInBodyAtPointInWorld(PointInWorld, NWU) - VelocityInBody;
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
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation);

    // Set the body velocity, expressed in the body reference frame
    Velocity VelocityInBody(1.,6.,4.);
    body->SetVelocityInBodyNoRotation(VelocityInBody,NWU);
    Test_AllGetVelocity(body,VelocityInBody, BodyAngularVelocity, is_orientation);

    // Set the body generalized velocity, expressed in the world reference frame
    body->SetGeneralizedVelocityInWorld(VelocityInWorld, BodyAngularVelocity, NWU);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation);

    // Set the body generalized velocity, expressed in the body reference frame
    body->SetGeneralizedVelocityInBody(VelocityInBody, BodyAngularVelocity, NWU);
    Test_AllGetVelocity(body,VelocityInBody, BodyAngularVelocity, is_orientation);

    //+++++Point Velocity Setters+++++//
    Position PointInWorld(5.,6.,7.); Position PointInBody(7.,6.,5.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,NWU);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInWorld"<<std::endl;
    Velocity VelocityInWorldAtPointInWorld(3.,1.,5.);
    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,BodyAngularVelocity,NWU);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInWorld, BodyAngularVelocity, is_orientation);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInBody"<<std::endl;
    Velocity VelocityInWorldAtPointInBody(2.,1.,8.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,BodyAngularVelocity,NWU);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInBody, BodyAngularVelocity, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInWorld"<<std::endl;
    Velocity VelocityInBodyAtPointInWorld(8.,1.,9.);
    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,BodyAngularVelocity,NWU);

    Test_AllGetVelocity(body, VelocityInBodyAtPointInWorld, BodyAngularVelocity, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInBody"<<std::endl;
    Velocity VelocityInBodyAtPointInBody(1.,1.,5.);
    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,BodyAngularVelocity,NWU);

    Test_AllGetVelocity(body, VelocityInBodyAtPointInBody, BodyAngularVelocity, is_orientation);

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
//    std::cout<<"SetVelocityInWorldNoRotation"<<std::endl;
    body->SetVelocityInWorldNoRotation(VelocityInWorld,NWU);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation);

    // Set the body velocity, expressed in the body reference frame
    const Velocity VelocityInBody(1.,6.,4.);
    VelocityInWorld = EasyRotate(VelocityInBody);
//    std::cout<<"SetVelocityInBodyNoRotation"<<std::endl;
    body->SetVelocityInBodyNoRotation(VelocityInBody,NWU);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation);

    // Set the body generalized velocity, expressed in the world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorld"<<std::endl;
    body->SetGeneralizedVelocityInWorld(VelocityInWorld, BodyAngularVelocity, NWU);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation);

    // Set the body generalized velocity, expressed in the body reference frame
//    std::cout<<"SetGeneralizedVelocityInBody"<<std::endl;
    body->SetGeneralizedVelocityInBody(VelocityInBody, BodyAngularVelocity, NWU);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation);

    //+++++Point Velocity Setters+++++//
    Position PointInWorld(5.,6.,7.); Position PointInBody(7.,6.,5.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,NWU);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInWorld"<<std::endl;
    Velocity VelocityInWorldAtPointInWorld(3.,1.,5.);
    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,BodyAngularVelocity,NWU);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInWorld, BodyAngularVelocity, is_orientation);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInBody"<<std::endl;
    Velocity VelocityInWorldAtPointInBody(2.,1.,8.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,BodyAngularVelocity,NWU);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInBody, BodyAngularVelocity, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInWorld"<<std::endl;
    const Velocity VelocityInBodyAtPointInWorld(8.,1.,9.);
    VelocityInWorldAtPointInWorld = EasyRotate(VelocityInBodyAtPointInWorld);
    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,BodyAngularVelocity,NWU);
    Test_AllGetVelocity(body, VelocityInWorldAtPointInWorld, BodyAngularVelocity, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in body reference frame
    const Velocity VelocityInBodyAtPointInBody(1.,1.,5.);
    VelocityInWorldAtPointInBody = EasyRotate(VelocityInBodyAtPointInBody);
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInBody"<<std::endl;
    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,BodyAngularVelocity,NWU);

    Test_AllGetVelocity(body, VelocityInWorldAtPointInBody, BodyAngularVelocity, is_orientation);
}



TEST(FrBodyTest,TranslationalVelocityWithAngularVelocity){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOG(OrigLocalCOGPos, NWU);

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
    body->SetGeneralizedVelocityInWorld(VelocityInWorld, BodyAngularVelocity, NWU);
    Test_AllGetVelocity(body,VelocityInWorld, BodyAngularVelocity, is_orientation);

    // Set the body generalized velocity, expressed in the body reference frame
    body->SetGeneralizedVelocityInBody(VelocityInBody, BodyAngularVelocity, NWU);
    Test_AllGetVelocity(body,VelocityInBody, BodyAngularVelocity, is_orientation);

    //+++++Point Velocity Setters+++++//
    Position PointInWorld(5.,6.,7.); Position PointInBody(7.,6.,5.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,NWU);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInWorld"<<std::endl;
    Velocity VelocityInWorldAtPointInWorld(3.,1.,5.);
    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,BodyAngularVelocity,NWU);

    VelocityInWorld = VelocityInWorldAtPointInWorld - BodyAngularVelocity.cross(PointInWorld - body->GetPosition(NWU));
    Test_AllGetVelocity(body, VelocityInWorld, BodyAngularVelocity, is_orientation);

    // Test Setter for the generalized velocity expressed in the world reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInWorldAtPointInBody"<<std::endl;
    Velocity VelocityInWorldAtPointInBody(2.,1.,8.);
    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,BodyAngularVelocity,NWU);

    VelocityInWorld = VelocityInWorldAtPointInBody -
            BodyAngularVelocity.cross(body->GetPointPositionInWorld(PointInBody,NWU) - body->GetPosition(NWU));
    Test_AllGetVelocity(body, VelocityInWorld, BodyAngularVelocity, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in world reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInWorld"<<std::endl;
    Velocity VelocityInBodyAtPointInWorld(8.,1.,9.);
    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,BodyAngularVelocity,NWU);

    VelocityInWorld = VelocityInBodyAtPointInWorld - BodyAngularVelocity.cross(PointInWorld - body->GetPosition(NWU));
    Test_AllGetVelocity(body, VelocityInWorld, BodyAngularVelocity, is_orientation);

    // Test Setter for the generalized velocity expressed in the body reference frame,
    // at a point expressed in body reference frame
//    std::cout<<"SetGeneralizedVelocityInBodyAtPointInBody"<<std::endl;
    Velocity VelocityInBodyAtPointInBody(1.,1.,5.);
    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,BodyAngularVelocity,NWU);

    VelocityInWorld = VelocityInBodyAtPointInBody -
                      BodyAngularVelocity.cross(body->GetPointPositionInWorld(PointInBody,NWU) - body->GetPosition(NWU));
    Test_AllGetVelocity(body, VelocityInWorld, BodyAngularVelocity, is_orientation);

}


TEST(FrBodyTest,ProjectVectorMethods){

    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

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

    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2 ;
    body->SetRotation(TotalRotation);

    const Position NWUWorldPos(1,2,3);

    Position NWUBodyPos = body->ProjectVectorInBody(NWUWorldPos,NWU);
    Position testPosition = NWUBodyPos - EasyRotateInv(NWUWorldPos);
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