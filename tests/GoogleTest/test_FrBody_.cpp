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
        Position testPosition = OrigAbsPos-GetAbsPosition(NWU);
        EXPECT_TRUE(testPosition.isZero());

        //-----------------COG-----------------//

        // Set the COG position, expressed in local body reference frame
        Position OrigLocalCOGPos(2.,3.,4.);
        SetCOGLocalPosition(OrigLocalCOGPos, false, NWU);

        // Test the Getters for the COG position, expressed in world reference frame
        testPosition = GetCOGAbsPosition(NWU)-(OrigLocalCOGPos + OrigAbsPos);
        EXPECT_TRUE(testPosition.isZero());

        // Test the Getters for the COG position, expressed in local body reference frame
        testPosition = GetCOGLocalPosition(NWU)-OrigLocalCOGPos;
        EXPECT_TRUE(testPosition.isZero());

        //-----------------Fixed Point-----------------//
        // Test for the getter for the local position of a point expressed in the world reference frame
        Position OrigAbsPosLocalPoint(4.,5,.6);
        testPosition = GetLocalPositionOfAbsPoint(OrigAbsPosLocalPoint,NWU)-(OrigAbsPosLocalPoint - OrigAbsPos);
        EXPECT_TRUE(testPosition.isZero());


        //-----------------Orientation-----------------//
        // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
        FrRotation_ OrigAbsRot; OrigAbsRot.SetCardanAngles_DEGREES(1.,2.,3.,NWU);
        SetAbsRotation(OrigAbsRot);
        SetCardanAngles_DEGREES(1.,2.,3.,NWU);

        // Test of the getter for the absolute orientation (expressed in the world reference frame)
        EXPECT_TRUE(OrigAbsRot==GetAbsRotation());

        // Test of the setter using cardan angles
        SetCardanAngles_DEGREES(1.,2.,3.,NWU);
        EXPECT_TRUE(OrigAbsRot==GetAbsRotation());

        // Rotation to an easy transformation
        SetCardanAngles_DEGREES(90.,0.,0.,NWU);

        // Test that the orientation of the body changed the previous getter result
        Position OrigPosTest = (OrigAbsPosLocalPoint - OrigAbsPos);
        auto OrigPosTestX = OrigPosTest.GetX();
        auto OrigPosTestY = OrigPosTest.GetY();
        auto OrigPosTestZ = OrigPosTest.GetZ();
        Position NewOrigPos (OrigPosTestX, -OrigPosTestZ, OrigPosTestY);
        testPosition = GetLocalPositionOfAbsPoint(OrigAbsPosLocalPoint,NWU)-NewOrigPos;
        EXPECT_TRUE(testPosition.isZero());

        
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