//
// Created by Lucas Letournel on 16/11/18.
//


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





TEST(FrQuaternion,Quaternion) {
    FRAME_CONVENTION fc = NED;
    double eps = 1e-8;

    FrUnitQuaternion_ Quat;
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

    // Test of SetNullRotation
    Quat.SetNullRotation();
    EXPECT_TRUE(Quat == FrUnitQuaternion_(1.,0.,0.,0.,fc));

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
    EXPECT_NEAR(0,((Quat.GetXAxis(fc)).cross(Direction(1,0,0))).norm(),eps);
    EXPECT_NEAR(0,((Quat.GetYAxis(fc)).cross(Direction(0,0,1))).norm(),eps);
    EXPECT_NEAR(0,((Quat.GetZAxis(fc)).cross(Direction(0,1,0))).norm(),eps);

    testDirection = Quat.GetXAxis(fc) - Direction(1,0,0);
    EXPECT_TRUE(testDirection.isZero());
    testDirection = Quat.GetYAxis(NWU) - Direction(0,0,1);
    EXPECT_TRUE(testDirection.isZero());
    testDirection = Quat.GetZAxis(NWU) - Direction(0,-1,0);
    EXPECT_TRUE(testDirection.isZero());

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

TEST(FrRotation,Rotation){
    FRAME_CONVENTION fc = NED;
    double eps = 1e-8;

    // Test SetNullRotation
    FrRotation_ NullRotation;
    NullRotation.SetNullRotation();

    FrRotation_ Rotation;
    EXPECT_TRUE(Rotation == NullRotation);

    // Test SetAxisAngle & associated constructor
    Direction Axis(1.,2.,3.); Axis.normalize();
    double angle = 0.05;
    FrRotation_ AxisAngleRot(Axis, angle, fc);
//    AxisAngleRot.SetAxisAngle(Axis, angle, fc);

    // Test GetAxisAngle
    Direction testAxis;     //AxisAngleRot.GetAxis(testAxis,fc);
    double testAngle;       //AxisAngleRot.GetAngle(testAngle);
    AxisAngleRot.GetAxisAngle(testAxis,testAngle,fc);
    testAxis -= Axis;
    EXPECT_TRUE(testAxis.isZero());
    EXPECT_NEAR(angle,testAngle,eps);

    // Test Setter from a FrUnitQuaternion
    FrUnitQuaternion_ AxisAngleQuat(Axis,angle,fc);
    Rotation.Set(AxisAngleQuat);
    EXPECT_TRUE(Rotation == AxisAngleRot);

    // Test copy Constructor
    auto NewRot(Rotation);
    EXPECT_TRUE(Rotation == NewRot);

    // Rotation to an easy transformation
    FrRotation_ XRotation; XRotation.SetCardanAngles_DEGREES(90.,0.,0.,fc);
    FrRotation_ YRotation; YRotation.SetCardanAngles_DEGREES(0.,90.,0.,fc);
    FrRotation_ TotalRotation = XRotation*YRotation ;

    // Test GetCardanAngles
    double rx, ry, rz;
    TotalRotation.GetCardanAngles_DEGREES(rx,ry,rz,fc);
    Direction testDirection(rx,ry,rz);
    testDirection -= Direction(90.,0.,90.);
    EXPECT_TRUE(testDirection.isZero());

    // Test GetQuaternion
    FrUnitQuaternion_ xRotQuat(Direction(1,0,0),90.*DEG2RAD,fc);
    auto xQuat = XRotation.GetQuaternion();
    EXPECT_TRUE(xRotQuat == xQuat);

    // Test on GetXAxis, GetYAxis, GetZAxis
    testDirection = xRotQuat.GetXAxis(NWU) - Direction(1,0,0);
    EXPECT_TRUE(testDirection.isZero());
    testDirection = xRotQuat.GetYAxis(NWU) - Direction(0,0,1);
    EXPECT_TRUE(testDirection.isZero());
    testDirection = xRotQuat.GetZAxis(NWU) - Direction(0,-1,0);
    EXPECT_TRUE(testDirection.isZero());

    // Test Rotate
    testDirection.Set(0.,1.,0.);
    testDirection = XRotation.Rotate(testDirection,fc);
    testDirection -= Direction(0.,0.,1.);
    EXPECT_TRUE(testDirection.isZero());

    // Test GetRotationMatrix and GetInverseRotationMatrix
    auto AxisAngleMatrix = AxisAngleRot.GetRotationMatrix();
    auto AxisAnglMatrixInv = AxisAngleRot.GetInverseRotationMatrix();

    // Test LeftMultiplyInverse
    auto testMatrix = AxisAngleRot.LeftMultiplyInverse(AxisAngleMatrix);
    EXPECT_TRUE(testMatrix.IsIdentity());

    // Test RightMultiplyInverse
    testMatrix = AxisAngleRot.RightMultiplyInverse(AxisAngleMatrix);
    EXPECT_TRUE(testMatrix.IsIdentity());

    // Test LeftMultiply
    testMatrix = AxisAngleRot.LeftMultiply(AxisAnglMatrixInv);
    EXPECT_TRUE(testMatrix.IsIdentity());

    // Test RightMultiply
    testMatrix = AxisAngleRot.RightMultiply(AxisAnglMatrixInv);
    EXPECT_TRUE(testMatrix.IsIdentity());

    // Test GetRotationMatrix and RightMultiply
    EXPECT_TRUE(TotalRotation.GetRotationMatrix().IsEqual(YRotation.RightMultiply(XRotation.GetRotationMatrix())));


    // Test RotAxisAngle_DEGREES
    FrRotation_ testRotation = XRotation.RotAxisAngle_DEGREES(Direction(0,1,0),90.,fc);
    EXPECT_TRUE(testRotation == TotalRotation);

//    testRotation = XRotation.RotY_DEGREES(90.,fc);
//    EXPECT_TRUE(testRotation == TotalRotation);
}