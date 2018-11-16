//
// Created by Lucas Letournel on 16/11/18.
//


#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrQuaternion,Quaternion) {
    FRAME_CONVENTION fc = NED;
    double eps = 1e-8;

    FrUnitQuaternion_ Quat;

    // Test of SetNullRotation
    Quat.SetNullRotation();
    EXPECT_TRUE(Quat == FrUnitQuaternion_(1.,0.,0.,0.,fc));

    // Test IsRotation(), Normalize() using the assert contained in this setter
    // Test Setter and Getter using doubles
    //      Set
    Quat.Set(1,2,3,4,true,fc);
    double q0,q1,q2,q3; double length  = sqrt(1.+4.+9.+16.);
    //      Get
    Quat.Get(q0,q1,q2,q3,fc);
    EXPECT_NEAR(1./length,q0,eps);
    EXPECT_NEAR(2./length,q1,eps);
    EXPECT_NEAR(3./length,q2,eps);
    EXPECT_NEAR(4./length,q3,eps);

    // Test of the Setter and Getter, using direction and angle
    Direction QuatDir(5.,6.,1.); QuatDir.normalize();
    double QuatAngle = 0.05;

    //      Set
    Quat.Set(QuatDir,QuatAngle,fc);

    //      Get
    Direction testDirection, resDirection ; double testQuatAngle;
    Quat.Get(resDirection,testQuatAngle,fc);
    // Test on the angle
    EXPECT_NEAR(QuatAngle,testQuatAngle,eps);
    // Test on the direction
    testDirection = resDirection - QuatDir;
    EXPECT_TRUE(testDirection.isZero());
    if (not(testDirection.isZero())) {
        std::cout<<QuatDir<<std::endl;
        std::cout<<resDirection<<std::endl;
    }

    // Test of GetXAxis, GetYAxis, GetZAxis
    QuatDir.Set(1.,0.,0.); QuatAngle = 90.*DEG2RAD;
    Quat.Set(QuatDir,QuatAngle,fc);
    EXPECT_NEAR(0,((Quat.GetXAxis(fc)).cross(QuatDir)).norm(),eps);
    EXPECT_NEAR(1,((Quat.GetYAxis(fc)).cross(QuatDir)).norm(),eps);
    EXPECT_NEAR(1,((Quat.GetZAxis(fc)).cross(QuatDir)).norm(),eps);

    // Test of the Copy Constructor
    FrUnitQuaternion_ CopyQuat(Quat); //lol copyQuat
    EXPECT_TRUE(CopyQuat == Quat);

    FrUnitQuaternion_ xRot(Direction(1.,0.,0.),90.*DEG2RAD,fc);
    FrUnitQuaternion_ yRot(Direction(0.,1.,0.),90.*DEG2RAD,fc);

    // Test GetInverse
    auto TotalRot  = xRot;
    TotalRot *= xRot.GetInverse();
    EXPECT_TRUE(TotalRot == FrUnitQuaternion_(1,0,0,0,fc));

    // Test Inverse
    TotalRot = xRot;
    TotalRot.Inverse();
    TotalRot *= xRot;
    EXPECT_TRUE(TotalRot == FrUnitQuaternion_(1,0,0,0,fc));

    // Test Rotate
    testDirection.Set(0.,1.,0.);
    testDirection = xRot.Rotate(testDirection,fc);
    testDirection -= Direction(0.,0.,1.);
    EXPECT_TRUE(testDirection.isZero());

    // Test GetRotationMatrix and GetInverseRotationMatrix
    auto xRotMatrix = xRot.GetRotationMatrix();
    auto xRotMatrixInv = xRot.GetInverseRotationMatrix();

    // Test LeftMultiplyInverse
    auto testMatrix = xRot.LeftMultiplyInverse(xRotMatrix);
    EXPECT_TRUE(testMatrix.IsIdentity());

    // Test RightMultiplyInverse
    testMatrix = xRot.RightMultiplyInverse(xRotMatrix);
    EXPECT_TRUE(testMatrix.IsIdentity());

    // Test LeftMultiply
    testMatrix = xRot.LeftMultiply(xRotMatrixInv);
    EXPECT_TRUE(testMatrix.IsIdentity());

    // Test RightMultiply
    testMatrix = xRot.RightMultiply(xRotMatrixInv);
    EXPECT_TRUE(testMatrix.IsIdentity());

    // Test GetRotationMatrix and RightMultiply
    auto NewRot = xRot * yRot;
    EXPECT_TRUE(NewRot.GetRotationMatrix().IsEqual(yRot.RightMultiply(xRotMatrix)));


}