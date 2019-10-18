//
// Created by lletourn on 18/10/19.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;


TEST(FrLinkTest, Base) {

  FRAME_CONVENTION fc = NWU;

  FrOffshoreSystem system;

  // nody creation
  auto body = system.NewBody();

  // body position
  Position randPos; randPos.setRandom();
  body->SetPosition(randPos, fc);

  // bodyNode creation
  auto bodyNode = body->NewNode();
  Position bodyNodePosInBody; bodyNodePosInBody.setRandom();
  bodyNode->SetPositionInBody(bodyNodePosInBody, fc);

  // worldNode Creation
  auto worldNode = system.GetWorldBody()->NewNode();
  Position worldNodePosInBody; worldNodePosInBody.setRandom();
  worldNode->SetPositionInBody(worldNodePosInBody, fc);

  // link  creation
  auto fixedLink = make_fixed_link(worldNode, bodyNode, &system);

  // initialization and assembly
  system.Initialize();
  system.DoAssembly();

  // Tests
  EXPECT_TRUE(fixedLink->GetNode1() == worldNode);
  EXPECT_TRUE(fixedLink->GetNode2() == bodyNode);

  auto relFrame = fixedLink->GetNode1FrameWRTNode2Frame();
  EXPECT_TRUE(relFrame.GetPosition(fc).isZero());
  EXPECT_TRUE(relFrame.GetRotation().GetRotationMatrix().isIdentity());

  relFrame = fixedLink->GetNode2FrameWRTNode1Frame();
  EXPECT_TRUE(relFrame.GetPosition(fc).isZero());
  EXPECT_TRUE(relFrame.GetRotation().GetRotationMatrix().isIdentity());





}