//
// Created by lletourn on 18/10/19.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;


TEST(FrLinkTest, force_accessors) {

  FRAME_CONVENTION fc = NWU;

  FrOffshoreSystem system;

  auto WBody = system.GetWorldBody();
  makeItBox(WBody, 2., 0.2, 0.2, 1.);
//  makeItSphere(WBody, 1., 1.);
  WBody->SetColor(Yellow);
  WBody->AllowCollision(false);

  // nody creation
  auto body = system.NewBody();
  makeItBox(body, 2., 0.2, 0.2, 1.);
//  makeItSphere(body, 1., 1.);
  body->SetColor(Green);
  body->AllowCollision(false);

  // body position
  Position randPos; randPos.setRandom();
  body->SetPosition(randPos, fc);

  // bodyNode creation
  auto bodyNode = body->NewNode();
  Position bodyNodePosInBody; bodyNodePosInBody.setRandom();
  bodyNodePosInBody = {-1.,0.,0.};
  bodyNode->SetPositionInBody(bodyNodePosInBody, fc);
  Direction direction; direction.setRandom(); direction.normalize();
  bodyNode->SetOrientationInBody(FrRotation(direction, 0.35894, fc));

  // worldNode Creation
  auto worldNode = WBody->NewNode();
  Position worldNodePosInBody; worldNodePosInBody.setRandom();
  worldNodePosInBody = {1.,0.,0.};
  worldNode->SetPositionInBody(worldNodePosInBody, fc);
  Direction direction2; direction2.setRandom(); direction2.normalize();
  bodyNode->SetOrientationInBody(FrRotation(direction2, 0.19708, fc));

  // link  creation
  auto fixedLink = make_fixed_link(worldNode, bodyNode, &system);

  // initialization and assembly
  system.Initialize();
  system.DoAssembly();
//  fixedLink->Update(0.); // generating cached values

  system.GetStaticAnalysis()->SetNbIteration(20);
  system.GetStaticAnalysis()->SetNbSteps(50);

  system.SolveStaticWithRelaxation();
  system.Visualize(5.);

  // Tests nodes frame
  EXPECT_TRUE(fixedLink->GetNode1() == worldNode);
  EXPECT_TRUE(fixedLink->GetNode2() == bodyNode);

  auto relFrame = fixedLink->GetNode1FrameWRTNode2Frame();
  EXPECT_TRUE(relFrame.GetPosition(fc).isZero(1e-5));
  EXPECT_TRUE(relFrame.GetRotation().GetRotationMatrix().isIdentity(1e-2));
  if (!relFrame.GetPosition(fc).isZero(1e-5) or !relFrame.GetRotation().GetRotationMatrix().isIdentity(1e-2)) {
    std::cout<<relFrame<<std::endl;
  }

  relFrame = fixedLink->GetNode2FrameWRTNode1Frame();
  EXPECT_TRUE(relFrame.GetPosition(fc).isZero(1e-5));
  EXPECT_TRUE(relFrame.GetRotation().GetRotationMatrix().isIdentity(1e-2));

  // Tests force and torque accessors
  auto gravityInWorld = GeneralizedForceTorsor(
      Force(0.,0.,-body->GetMass() * system.GetGravityAcceleration()),
      Torque(),
      body->GetCOGPositionInWorld(fc), fc);

  Force testForce = worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnNode1(fc), fc) - gravityInWorld.GetForce();
  EXPECT_TRUE(testForce.isZero(1e-8));
  if (!testForce.isZero(1e-8)) {
    std::cout<<fixedLink->GetLinkReactionForceOnNode1(fc)<<std::endl;
    std::cout<<gravityInWorld.GetForce()<<std::endl;
  }

  Torque testTorque = worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnNode1(fc), fc) - gravityInWorld.GetTorqueAtPoint(worldNode->GetPositionInWorld(fc), fc);
  EXPECT_TRUE(testTorque.isZero(1e-8));
  if (!testForce.isZero(1e-8)) {
    std::cout<<fixedLink->GetLinkReactionTorqueOnNode1(fc)<<std::endl;
    std::cout<<gravityInWorld.GetTorqueAtPoint(worldNode->GetPositionInWorld(fc),fc)<<std::endl;
  }

  testForce = worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnNode1(fc), fc)
      + bodyNode->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnNode2(fc), fc);
  EXPECT_TRUE(testForce.isZero(1e-4));
  if (!testForce.isZero(1e-4)) {
    std::cout<<fixedLink->GetLinkReactionForceOnNode1(fc)<<std::endl;
    std::cout<<fixedLink->GetLinkReactionForceOnNode2(fc)<<std::endl;
  }

  testTorque = worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnNode1(fc), fc)
      + bodyNode->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnNode2(fc), fc);
  EXPECT_TRUE(testTorque.isZero(1e-4));
  if (!testTorque.isZero(1e-4)) {
    std::cout<<fixedLink->GetLinkReactionTorqueOnNode1(fc)<<std::endl;
    std::cout<<fixedLink->GetLinkReactionTorqueOnNode2(fc)<<std::endl;
  }

  testForce = WBody->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnBody1(fc), fc) - gravityInWorld.GetForce();
  EXPECT_TRUE(testForce.isZero(1e-8));

  testTorque = WBody->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnBody1AtCOG(fc), fc) - gravityInWorld.GetTorqueAtPoint(WBody->GetCOGPositionInWorld(fc), fc);
  EXPECT_TRUE(testTorque.isZero(1e-8));

  testForce = body->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnBody2(fc), fc) + gravityInWorld.GetForce();
  EXPECT_TRUE(testForce.isZero(1e-8));

  testTorque = body->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnBody2AtCOG(fc), fc) + gravityInWorld.GetTorqueAtPoint(body->GetCOGPositionInWorld(fc), fc);
  EXPECT_TRUE(testTorque.isZero(1e-8));



}