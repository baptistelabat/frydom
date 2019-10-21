//
// Created by lletourn on 18/10/19.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;


TEST(FrLinkTest, FrLinkTest_Fixed_force_Test) {

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
//  system.Visualize(5.);

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
    std::cout<<worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnNode1(fc), fc)<<std::endl;
    std::cout<<gravityInWorld.GetForce()<<std::endl;
  }

  Torque testTorque = worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnNode1(fc), fc) - gravityInWorld.GetTorqueAtPoint(worldNode->GetPositionInWorld(fc), fc);
  EXPECT_TRUE(testTorque.isZero(1e-8));
  if (!testForce.isZero(1e-8)) {
    std::cout<<worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnNode1(fc), fc)<<std::endl;
    std::cout<<gravityInWorld.GetTorqueAtPoint(worldNode->GetPositionInWorld(fc),fc)<<std::endl;
  }

  testForce = worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnNode1(fc), fc)
      + bodyNode->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnNode2(fc), fc);
  EXPECT_TRUE(testForce.isZero(1e-4));
  if (!testForce.isZero(1e-4)) {
    std::cout<<worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnNode1(fc), fc)<<std::endl;
    std::cout<<bodyNode->ProjectVectorInWorld(fixedLink->GetLinkReactionForceOnNode2(fc), fc)<<std::endl;
  }

  testTorque = worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnNode1(fc), fc)
      + bodyNode->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnNode2(fc), fc);
  EXPECT_TRUE(testTorque.isZero(1e-4));
  if (!testTorque.isZero(1e-4)) {
    std::cout<<worldNode->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnNode1(fc), fc)<<std::endl;
    std::cout<<bodyNode->ProjectVectorInWorld(fixedLink->GetLinkReactionTorqueOnNode2(fc), fc)<<std::endl;
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


TEST(FrLinkTest, FrLinkTest_Prismatic_velocity_Test) {

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
  Position randPos;
  randPos.setRandom();
  body->SetPosition(randPos, fc);

  // bodyNode creation
  auto bodyNode = body->NewNode();
//  Position bodyNodePosInBody;
//  bodyNodePosInBody.setRandom();
//  bodyNodePosInBody = {-1., 0., 0.};
//  bodyNode->SetPositionInBody(bodyNodePosInBody, fc);
//  Direction direction;
//  direction.setRandom();
//  direction.normalize();
  FrRotation bodyNodeRot(Direction(0., 1., 0.), 90 * DEG2RAD, fc);
  bodyNode->SetFrameInBody(FrFrame(Position(), bodyNodeRot, fc));

  // worldNode Creation
  auto worldNode = WBody->NewNode();
  Position worldNodePosInBody;
  worldNodePosInBody.setRandom();
  worldNodePosInBody = {1., 0., 0.};
  worldNode->SetPositionInBody(worldNodePosInBody, fc);
//  worldNode->SetOrientationInBody(FrRotation(Direction(0.,1.,0.), -45*DEG2RAD, fc));

  // link  creation
  auto link = make_prismatic_link(worldNode, bodyNode, &system);
  link->SetSpringDamper(10., 0.);

// initialization and assembly
  system.Initialize();
  system.DoAssembly();
//  link->Update(0.); // generating cached values
  system.SetTimeStep(0.01);

  bool irrlicht = false;

  if (irrlicht) {
    system.RunInViewer(10., 5.);
  } else {
    double time = 0.;
    while (time < 10.) {
      system.AdvanceTo(time);
      time += 0.01;
    }
  }

  // Tests nodes frame
  EXPECT_TRUE(link->GetNode1() == worldNode);
  EXPECT_TRUE(link->GetNode2() == bodyNode);

  FrFrame testFrame = worldNode->GetFrameInWorld().GetThisFrameRelativeTransform_WRT_OtherFrame(
      bodyNode->GetFrameInWorld());
  auto relFrame = link->GetNode1FrameWRTNode2Frame();

  EXPECT_TRUE(testFrame.IsApprox(relFrame));
  if (!testFrame.IsApprox(relFrame)) {
    std::cout << testFrame << std::endl;
    std::cout << relFrame << std::endl;
  }

  // test nodes velocity
  auto node2Velo = link->GetGeneralizedVelocityOfNode2WRTNode1(fc);

  GeneralizedVelocityTorsor node2Vel(
      body->GetVelocityInWorldAtPointInBody(bodyNode->GetNodePositionInBody(fc), fc),
      body->GetAngularVelocityInWorld(fc),
      bodyNode->GetPositionInWorld(fc), fc
  );

  bool test = worldNode->ProjectVectorInWorld(node2Velo.GetVelocity(), fc).isApprox(
      node2Vel.GetLinearVelocityAtPoint(bodyNode->GetPositionInWorld(fc), fc));
  EXPECT_TRUE(test);
  if (!test) {
    std::cout << worldNode->ProjectVectorInWorld(node2Velo.GetVelocity(), fc) << std::endl;
    std::cout << node2Vel.GetLinearVelocityAtPoint(bodyNode->GetPositionInWorld(fc), fc) << std::endl;
  }

  test = worldNode->ProjectVectorInWorld(node2Velo.GetAngularVelocity(), fc).isApprox(node2Vel.GetAngularVelocity());
  EXPECT_TRUE(test);
  if (!test) {
    std::cout << worldNode->ProjectVectorInWorld(node2Velo.GetAngularVelocity(), fc) << std::endl;
    std::cout << node2Vel.GetAngularVelocity() << std::endl;
  }

  auto node1Velo = link->GetGeneralizedVelocityOfNode1WRTNode2(fc);

  test = bodyNode->ProjectVectorInWorld(node1Velo.GetVelocity(), fc).isApprox(
      -worldNode->ProjectVectorInWorld(node2Velo.GetVelocity(), fc));
  EXPECT_TRUE(test);
  if (!test) {
    std::cout << bodyNode->ProjectVectorInWorld(node1Velo.GetVelocity(), fc) << std::endl;
    std::cout << -worldNode->ProjectVectorInWorld(node2Velo.GetVelocity(), fc) << std::endl;
  }

  test = bodyNode->ProjectVectorInWorld(node1Velo.GetAngularVelocity(), fc).isApprox(
      -worldNode->ProjectVectorInWorld(node2Velo.GetAngularVelocity(), fc));
  EXPECT_TRUE(test);
  if (!test) {
    std::cout << bodyNode->ProjectVectorInWorld(node1Velo.GetAngularVelocity(), fc) << std::endl;
    std::cout << -worldNode->ProjectVectorInWorld(node2Velo.GetAngularVelocity(), fc) << std::endl;
  }


//  relFrame = link->GetNode2FrameWRTNode1Frame();
//  EXPECT_TRUE(relFrame.GetPosition(fc).IsZero(1e-5));
//  EXPECT_TRUE(relFrame.GetRotation().GetRotationMatrix().isIdentity(1e-2));

}


TEST(FrLinkTest, FrLinkTest_Prismatic_force_Test) {

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
  Position randPos;
  randPos.setRandom();
  body->SetPosition(randPos, fc);

  // bodyNode creation
  auto bodyNode = body->NewNode();
//  Position bodyNodePosInBody;
//  bodyNodePosInBody.setRandom();
//  bodyNodePosInBody = {-1., 0., 0.};
//  bodyNode->SetPositionInBody(bodyNodePosInBody, fc);
//  Direction direction;
//  direction.setRandom();
//  direction.normalize();
  FrRotation bodyNodeRot(Direction(0.,1.,0.), 90*DEG2RAD, fc);
  bodyNode->SetFrameInBody(FrFrame(Position(), bodyNodeRot, fc));

  // worldNode Creation
  auto worldNode = WBody->NewNode();
  Position worldNodePosInBody;
  worldNodePosInBody.setRandom();
  worldNodePosInBody = {1., 0., 0.};
  worldNode->SetPositionInBody(worldNodePosInBody, fc);
  worldNode->SetOrientationInBody(FrRotation(Direction(0.,1.,0.), -45*DEG2RAD, fc));

  // link  creation
  auto link = make_prismatic_link(worldNode, bodyNode, &system);
  link->SetSpringDamper(10.,0.);

// initialization and assembly
  system.Initialize();
  system.DoAssembly();

  system.SetTimeStep(0.01);
  system.GetStaticAnalysis()->SetNbSteps(20);
  system.GetStaticAnalysis()->SetNbIteration(50);

  system.SolveStaticWithRelaxation();
//  system.RunInViewer();



  // Tests link force (not reaction, the component from stiffness/damping

  auto gravityInWorld = GeneralizedForceTorsor(
//      Force(0.,0.,-0.5*sqrt(2.)*body->GetMass() * system.GetGravityAcceleration()),
      Force(0.,0.,-body->GetMass() * system.GetGravityAcceleration()),
      Torque(),
      body->GetCOGPositionInWorld(fc), fc);

  Force testForce = worldNode->ProjectVectorInWorld(link->GetLinkForceOnBody1InFrame1AtOrigin1(fc), fc)
      + worldNode->ProjectVectorInWorld(link->GetLinkReactionForceOnNode1(fc), fc)
      - gravityInWorld.GetForce();
  EXPECT_TRUE(testForce.isZero(1e-2));
  if (!testForce.isZero(1e-2)) {
    std::cout<<worldNode->ProjectVectorInWorld(link->GetLinkForceOnBody1InFrame1AtOrigin1(fc), fc)<<std::endl;
    std::cout<<worldNode->ProjectVectorInWorld(link->GetLinkReactionForceOnNode1(fc), fc)<<std::endl;
    std::cout<<gravityInWorld.GetForce()<<std::endl;
  }



}


TEST(FrLinkTest, FrLinkTest_Revolute_force_Test) {

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
  Position randPos;
  randPos.setRandom();
  body->SetPosition(randPos, fc);

  // bodyNode creation
  auto bodyNode = body->NewNode();
//  Position bodyNodePosInBody;
//  bodyNodePosInBody.setRandom();
//  bodyNodePosInBody = {-1., 0., 0.};
//  bodyNode->SetPositionInBody(bodyNodePosInBody, fc);
//  Direction direction;
//  direction.setRandom();
//  direction.normalize();
  FrRotation bodyNodeRot(Direction(1.,0.,0.), 90*DEG2RAD, fc);
  bodyNode->SetFrameInBody(FrFrame(Position(-1., 0., 0.), bodyNodeRot, fc));

  // worldNode Creation
  auto worldNode = WBody->NewNode();
  Position worldNodePosInBody;
  worldNodePosInBody.setRandom();
  worldNodePosInBody = {1., 0., 0.};
  worldNode->SetPositionInBody(worldNodePosInBody, fc);
  worldNode->SetOrientationInBody(FrRotation(Direction(1.,0.,0.), 90*DEG2RAD, fc));

  // link  creation
  auto link = make_revolute_link(worldNode, bodyNode, &system);
  link->SetSpringDamper(10.,0.);

// initialization and assembly
  system.Initialize();
//  system.DoAssembly();

  system.SetTimeStep(0.01);
  system.GetStaticAnalysis()->SetNbSteps(10);
  system.GetStaticAnalysis()->SetNbIteration(100);

  system.SolveStaticWithRelaxation();
  system.RunInViewer(0.,5.);



  // Tests link force (not reaction, the component from stiffness/damping

  auto gravityInWorld = GeneralizedForceTorsor(
//      Force(0.,0.,-0.5*sqrt(2.)*body->GetMass() * system.GetGravityAcceleration()),
      Force(0.,0.,-body->GetMass() * system.GetGravityAcceleration()),
      Torque(),
      body->GetCOGPositionInWorld(fc), fc);

  Force testForce = worldNode->ProjectVectorInWorld(link->GetLinkForceOnBody1InFrame1AtOrigin1(fc), fc)
                    + worldNode->ProjectVectorInWorld(link->GetLinkReactionForceOnNode1(fc), fc)
                    - gravityInWorld.GetForce();
  EXPECT_TRUE(testForce.isZero(1e-2));
  if (!testForce.isZero(1e-2)) {
    std::cout<<worldNode->ProjectVectorInWorld(link->GetLinkForceOnBody1InFrame1AtOrigin1(fc), fc)<<std::endl;
    std::cout<<worldNode->ProjectVectorInWorld(link->GetLinkReactionForceOnNode1(fc), fc)<<std::endl;
    std::cout<<gravityInWorld.GetForce()<<std::endl;
  }



}