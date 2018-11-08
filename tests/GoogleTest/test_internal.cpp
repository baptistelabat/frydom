//
// Created by Lucas Letournel on 07/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;
using namespace frydom::internal;

TEST(Test_Internal,FrFrame){
    // Init a Position
    FrRotation_ myRotation;    myRotation.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    
    // Init a Rotation
    Position myPosition(1.,2.,3.);
    
    // Init a Frame from these Position and Rotation
    FrFrame_ FRyDoMFrame(myPosition, myRotation, NWU);
    
    // Test on the Getters for the Position and Rotation
    Position testPosition = FRyDoMFrame.GetPosition(NWU)-myPosition;
    EXPECT_TRUE(testPosition.isZero());
    EXPECT_TRUE(FRyDoMFrame.GetRotation()==myRotation);
    
    // Convert to ChFrame
    chrono::ChFrame<double> ChronoFrame = Fr2ChFrame(FRyDoMFrame);
    
    // Test Position returned by Chrono Frame
    chrono::ChVector<double> ChPosition = ChronoFrame.GetPos();
    EXPECT_DOUBLE_EQ(ChPosition.x(), myPosition.GetX());
    EXPECT_DOUBLE_EQ(ChPosition.y(), myPosition.GetY());
    EXPECT_DOUBLE_EQ(ChPosition.z(), myPosition.GetZ());
    
    // Test Quaternion returned by Chrono Frame
    chrono::ChQuaternion<double> ChQuaternion = ChronoFrame.GetRot();
    double q0,q1,q2,q3; myRotation.GetQuaternion().Get(q0,q1,q2,q3,NWU);
    EXPECT_DOUBLE_EQ(ChQuaternion.e0(), q0);
    EXPECT_DOUBLE_EQ(ChQuaternion.e1(), q1);
    EXPECT_DOUBLE_EQ(ChQuaternion.e2(), q2);
    EXPECT_DOUBLE_EQ(ChQuaternion.e3(), q3);
    
    // Convert back to FrFrame
    FrFrame_ BackToFRFrame = Ch2FrFrame(ChronoFrame);
    // Test on the Getters for the Position and Rotation
    testPosition = FRyDoMFrame.GetPosition(NWU)-myPosition;
    EXPECT_TRUE(testPosition.isZero());
    EXPECT_TRUE(FRyDoMFrame.GetRotation()==myRotation);
    
    
    

}

