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

TEST(FrFrame, PositionAndRotation) {
  // Frame Convention
  FRAME_CONVENTION fc = NWU;

  // Frame to test
  FrFrame frame;

  // Frame Position
  const Position framePosition(1., 2., 3.);
  Position testPosition;

  // Test on Setter for the position
  frame.SetPosition(1., 2., 3., fc);
  testPosition = frame.GetPosition(fc) - framePosition;
  EXPECT_TRUE(testPosition.isZero());
  if (not(testPosition.isZero())) {
    std::cout << frame.GetPosition(fc) << std::endl;
    std::cout << framePosition << std::endl;
  }

  // Test on Setter for the position
  frame.SetPosition(framePosition, fc);
  testPosition = frame.GetPosition(fc) - framePosition;
  EXPECT_TRUE(testPosition.isZero());
  if (not(testPosition.isZero())) {
    std::cout << frame.GetPosition(fc) << std::endl;
    std::cout << framePosition << std::endl;
  }

  // Test on SetNoTranslation
  frame.SetNoTranslation();
  testPosition = frame.GetPosition(fc) - Position(0., 0., 0.);
  EXPECT_TRUE(testPosition.isZero());
  if (not(testPosition.isZero())) {
    std::cout << frame.GetPosition(fc) << std::endl;
    std::cout << Position(0., 0., 0.) << std::endl;
  }

  // Test on SetIdentity, using GetRotation and GetQuaternion
  frame.SetIdentity();
  testPosition = frame.GetPosition(fc) - Position(0., 0., 0.);
  EXPECT_TRUE(testPosition.isZero());
  if (not(testPosition.isZero())) {
    std::cout << frame.GetPosition(fc) << std::endl;
    std::cout << Position(0., 0., 0.) << std::endl;
  }
  FrUnitQuaternion IdentityQuat(1., 0., 0., 0., fc);
  FrRotation IdentityRotation;
  IdentityRotation.Set(IdentityQuat);
  EXPECT_TRUE(frame.GetRotation() == IdentityRotation);
  EXPECT_TRUE(frame.GetQuaternion() == IdentityQuat);

  // Test on Setter for the rotation
  FrRotation frameRotation;
  frameRotation.SetCardanAngles_DEGREES(3., 2., 1., fc);
  frame.SetRotation(frameRotation);
  EXPECT_TRUE(frame.GetRotation() == frameRotation);

  // Test on SetNoRotation
  frame.SetNoRotation();
  frameRotation.SetCardanAngles_DEGREES(0., 0., 0., fc);
  EXPECT_TRUE(frame.GetRotation() == frameRotation);


  //Test of the Rot methods with local axes.
  //      Rotation to an easy transformation
  FrRotation Rotation1;
  Rotation1.SetCardanAngles_DEGREES(90., 0., 0., fc);
  FrRotation Rotation2;
  Rotation2.SetCardanAngles_DEGREES(0., 90., 0., fc);
  FrRotation Rotation3;
  Rotation3.SetCardanAngles_DEGREES(0., 0., 90., fc);
  FrRotation TotalRotation = Rotation1 * Rotation2 * Rotation3;
  frame.SetRotation(TotalRotation);

  //      Applying the inverse rotations using the Rot methods
  frame.RotZ_DEGREES(-90, fc, true);
  frame.RotY_DEGREES(-90, fc, true);
  frame.RotX_DEGREES(-90, fc, true);
  double q0, q1, q2, q3;
  frame.GetQuaternion().Get(q0, q1, q2, q3, fc);
  double abs = 1e-8;
  EXPECT_NEAR(1, q0, abs);
  EXPECT_NEAR(0., q1, abs);
  EXPECT_NEAR(0., q2, abs);
  EXPECT_NEAR(0., q3, abs);

  //Test of the Rot methods with world axes.
  Rotation3.SetCardanAngles_DEGREES(90., 0., 0., fc);
  TotalRotation = Rotation1 * Rotation2 * Rotation3;
  frame.SetRotation(TotalRotation);

  //      Applying the inverse rotations using the Rot methods
  frame.RotY_DEGREES(-90, fc, false);
  frame.RotZ_DEGREES(-90, fc, false);
  frame.RotX_DEGREES(-90, fc, false);
  frame.GetQuaternion().Get(q0, q1, q2, q3, fc);
  EXPECT_NEAR(1, q0, abs);
  EXPECT_NEAR(0., q1, abs);
  EXPECT_NEAR(0., q2, abs);
  EXPECT_NEAR(0., q3, abs);

  // Test of the SetRot methods
  Rotation3.SetCardanAngles_DEGREES(0., 0., 90., fc);
  TotalRotation = Rotation1 * Rotation2 * Rotation3;
  frame.SetRotX_DEGREES(90, fc);
  EXPECT_TRUE(frame.GetRotation() == Rotation1);
  frame.SetRotY_DEGREES(90, fc);
  EXPECT_TRUE(frame.GetRotation() == Rotation2);
  frame.SetRotZ_DEGREES(90, fc);
  EXPECT_TRUE(frame.GetRotation() == Rotation3);


  // Test of GetOtherFrameRelativeTransform_WRT_ThisFrame
  frame.SetNoRotation();

  FrFrame Transf2OtherFrame;
  Transf2OtherFrame.SetPosition(Position(5., 8., 2.), fc);
  Transf2OtherFrame.SetRotation(TotalRotation);

  FrFrame OtherFrame = Transf2OtherFrame * frame;

  testPosition = OtherFrame.GetPosition(fc) - (frame.GetPosition(fc) + Transf2OtherFrame.GetPosition(fc));
  EXPECT_TRUE(testPosition.isZero());
  if (not(testPosition.isZero())) {
    std::cout << frame.GetPosition(fc) << std::endl;
    std::cout << Transf2OtherFrame.GetPosition(fc) << std::endl;
    std::cout << OtherFrame.GetPosition(fc) << std::endl;
  }
  EXPECT_TRUE(OtherFrame.GetRotation() == TotalRotation);

  auto testTransf = frame.GetOtherFrameRelativeTransform_WRT_ThisFrame(OtherFrame);
  testPosition = testTransf.GetPosition(fc) - Transf2OtherFrame.GetPosition(fc);
  EXPECT_TRUE(testPosition.isZero());
  EXPECT_TRUE(testTransf.GetRotation() == Transf2OtherFrame.GetRotation());


  // Test of GetThisFrameRelativeTransform_WRT_OtherFrame, using GetInverse()
  auto testTransfInv = frame.GetThisFrameRelativeTransform_WRT_OtherFrame(OtherFrame);
  testPosition = testTransfInv.GetPosition(fc) - (testTransf.GetInverse()).GetPosition(fc);
  EXPECT_TRUE(testPosition.isZero());
  EXPECT_TRUE(testTransfInv.GetRotation() == (testTransf.GetInverse()).GetRotation());

  // Test of Inverse()
  testTransf.Inverse();
  testPosition = testTransfInv.GetPosition(fc) - testTransf.GetPosition(fc);
  EXPECT_TRUE(testPosition.isZero());
  EXPECT_TRUE(testTransfInv.GetRotation() == testTransf.GetRotation());

}

TEST(FrFrame, ProjectVectors) {
  // Frame Convention
  FRAME_CONVENTION fc = NED;

  // Frame to test
  FrFrame frame;

  frame.RotZ_DEGREES(90., fc, false);

  Direction north = NORTH(fc);
  Direction west = WEST(fc);

//    std::cout<<"north"<<frame.ProjectVectorFrameInParent(north,fc)<<std::endl;
//    std::cout<<"west"<<frame.ProjectVectorFrameInParent(west,fc)<<std::endl;

  Direction testDirection = frame.ProjectVectorFrameInParent(north, fc) - Direction(0., 1., 0.);
  EXPECT_TRUE(testDirection.isZero());

  testDirection = frame.ProjectVectorParentInFrame(Direction(0., 1., 0.), fc) - north;
  EXPECT_TRUE(testDirection.isZero());
}