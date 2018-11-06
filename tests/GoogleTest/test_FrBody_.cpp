//
// Created by Lucas Letournel on 05/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;


class TestFrBody_ :public FrBody_ {
public:
    void TestSettersGettersPosition(){
        Position OrigAbsPos(1.,2,.3);
        SetAbsPosition(OrigAbsPos,NWU);

        // Test of the getter for the absolute position (expressed in the world reference frame)
//        auto absPosNWU = GetAbsPosition(NWU);
//        EXPECT_EQ( (OrigAbsPos-absPosNWU).norm(), 0. );
        Position testPosition = OrigAbsPos-GetAbsPosition(NWU);
        EXPECT_TRUE(testPosition.isZero());
//        EXPECT_EQ( ().norm(), 0. );

        // Set the COG position, expressed in local body reference frame
        Position OrigLocalCOGPos(2.,3.,4.);
        SetCOGLocalPosition(OrigLocalCOGPos, false, NWU);

        // Test the Getters for the COG position, expressed in world reference frame
        testPosition = GetCOGAbsPosition(NWU)-(OrigLocalCOGPos + OrigAbsPos);
        EXPECT_TRUE(testPosition.isZero());
//        EXPECT_EQ( (GetCOGAbsPosition(NWU)-(OrigLocalCOGPos + OrigAbsPos)).norm(), 0. );
        // Test the Getters for the COG position, expressed in local body reference frame
        testPosition = GetCOGLocalPosition(NWU)-OrigLocalCOGPos;
        EXPECT_TRUE(testPosition.isZero());
//        EXPECT_EQ( (GetCOGLocalPosition(NWU)-OrigLocalCOGPos).norm(), 0. );


//        // Set the COG position, expressed in world reference frame
//        Position OrigAbsCOGPos(2.,3.,4.);
//        SetCOGAbsPosition(OrigAbsCOGPos, NWU);
//
//        // Test the Getters for the COG position, expressed in world reference frame
//        EXPECT_EQ( (GetCOGAbsPosition(NWU)-OrigAbsCOGPos).norm(), 0. );
//        // Test the Getters for the COG position, expressed in local body reference frame
//        Position truc = OrigAbsCOGPos - OrigAbsPos;
//        EXPECT_EQ( (GetCOGLocalPosition(NWU)-truc).norm(), 0. );
//        EXPECT_EQ(GetCOGLocalPosition(NWU).GetX(),truc.GetX());
//        EXPECT_EQ(GetCOGLocalPosition(NWU).GetY(),truc.GetY());
//        EXPECT_EQ(GetCOGLocalPosition(NWU).GetZ(),truc.GetZ());


        // Test for the getter for the local position of a point expressed in the world reference frame
        Position OrigAbsPosLocalPoint(4.,5,.6);
//        auto relPosNWU = GetLocalPositionOfAbsPoint(OrigAbsPosLocalPoint,NWU);
//        EXPECT_EQ( (relPosNWU-(OrigAbsPosLocalPoint - OrigAbsPos)).norm(), 0. );
        testPosition = GetLocalPositionOfAbsPoint(OrigAbsPosLocalPoint,NWU)-(OrigAbsPosLocalPoint - OrigAbsPos);
        EXPECT_TRUE(testPosition.isZero());


        // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
        FrRotation_ OrigAbsRot; OrigAbsRot.SetCardanAngles_DEGREES(1.,2.,3.,NWU);
        SetAbsRotation(OrigAbsRot);

        // Test of the getter for the absolute orientation (expressed in the world reference frame)
        auto absRotNWU = GetAbsRotation();
//        EXPECT_EQ( (absRotNWU-OrigAbsRot).norm(), 0. );

        // Test that the orientation of the body changed the previous getter result
//        relPosNWU = GetLocalPositionOfAbsPoint(OrigAbsPosLocalPoint,NWU);
//        EXPECT_NE( (relPosNWU-(OrigAbsPosLocalPoint - OrigAbsPos)).norm(), 0. );
        testPosition = GetLocalPositionOfAbsPoint(OrigAbsPosLocalPoint,NWU)-(OrigAbsPosLocalPoint - OrigAbsPos);
        EXPECT_FALSE(testPosition.isZero());

        
    };

    void TestSettersGettersCOGPosition(){
        Position OrigCOGAbsPos(7.,8,.9);
        SetCOGAbsPosition(OrigCOGAbsPos,NWU);

        auto COGabsPosNWU = GetCOGAbsPosition(NWU);
        EXPECT_EQ(COGabsPosNWU.GetX(),OrigCOGAbsPos.GetX());
        EXPECT_EQ(COGabsPosNWU.GetY(),OrigCOGAbsPos.GetY());
        EXPECT_EQ(COGabsPosNWU.GetZ(),OrigCOGAbsPos.GetZ());
    };




    int Test_Smthg(){return 0;};
    int Test_SmthgElse(){return 100;};
};

TEST(FrBodyTest,TestSettersGettersPosition){
    auto body = std::make_shared<TestFrBody_>();
    body->TestSettersGettersPosition();
}


//TEST(FrBody_Test,test_SMTHG){
////    FrOffshoreSystem_ system;
//    // Defining the ship
//    auto body = std::make_shared<TestFrBody_>();
////    body->SetSmoothContact();
////    system.AddBody(body);
//    // Testeing something
//    EXPECT_EQ(body->Test_Smthg(),0);
//};
//
//
//TEST(FrBody_Test,test_SMTHGELSE){
////    FrOffshoreSystem_ system;
//    // Defining the ship
//    auto body = std::make_shared<TestFrBody_>();
////    body->SetSmoothContact();
////    system.AddBody(body);
//    // Testeing something
//    EXPECT_EQ(body->Test_SmthgElse(),100);
//};