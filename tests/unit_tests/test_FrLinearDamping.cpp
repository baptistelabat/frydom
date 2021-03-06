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

class TestLinearDamping : public ::testing::Test {

 protected:

  // ------------- test configuration -----------------
  double vx = 1.5;
  double vy = 0.9;
  double vz = 0.1;
  double vp = 0.05;
  double vq = -0.1;
  double vr = 0.6;

  Position bodyPositionInWorld = Position(10., 4.5, 2.3);
  Direction rotationDirection = Direction(0.1, 0.2, 0.9);
  double rotationAngle = M_PI / 6.;

  Position COGPosition = Position(0.2, 0.3, 0.5);

  FrLinearDamping::DampingMatrix damp;

  Velocity flowVelocity = Velocity(1., 1.5, 0.);
  // ----------------------------------------------------


  FrOffshoreSystem system;
  std::shared_ptr<FrBody> body;
  std::shared_ptr<FrLinearDamping> force;

  TestLinearDamping() : system("test_FrLinearDamping") {}

  void SetUp() override;

  void CheckForce(Force forceRef, Torque torqueRef) const {
    force->Update(0.);
    auto worldForce = force->GetForceInBody(NWU);
    auto bodyTorque = force->GetTorqueInBodyAtCOG(NWU);
    EXPECT_NEAR(forceRef.GetFx(), worldForce.GetFx(), 1E-8);
    EXPECT_NEAR(forceRef.GetFy(), worldForce.GetFy(), 1E-8);
    EXPECT_NEAR(forceRef.GetFz(), worldForce.GetFz(), 1E-8);
    EXPECT_NEAR(torqueRef.GetMx(), bodyTorque.GetMx(), 1E-8);
    EXPECT_NEAR(torqueRef.GetMy(), bodyTorque.GetMy(), 1E-8);
    EXPECT_NEAR(torqueRef.GetMz(), bodyTorque.GetMz(), 1E-8);
  }

};

void TestLinearDamping::SetUp() {

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(flowVelocity, NWU, GOTO);
  system.GetEnvironment()->GetTimeRamp()->SetActive(false);

  body = system.NewBody("body");
  body->SetFixedInWorld(true);
  body->SetPosition(bodyPositionInWorld, NWU);

  rotationDirection.normalize();

  body->SetRotation(FrUnitQuaternion(rotationDirection, rotationAngle, NWU));

  FrInertiaTensor InertiaTensor(1., 1., 1., 1., 0., 0., 0., COGPosition, NWU);
  body->SetInertiaTensor(InertiaTensor);

  body->SetGeneralizedVelocityInBodyAtPointInBody(body->GetCOG(NWU), Velocity(vx, vy, vz),
                                                  AngularVelocity(vp, vq, vr), NWU);

  force = make_linear_damping_force("TestLinearDamping", body, WATER, false);

  damp << 1.5, 0., 3.5, 6.5, 1.5, 0.,
      6.4, 5., 6.8, 5.5, 0., 0.,
      1.5, 6.5, 78, 1.5, 3., 5.,
      1.6, 0., 3.5, 4.5, 5.5, 0.,
      1.5, 5., 6.4, 4.8, 0., 0.,
      1.5, 6.5, 8, 5.5, 6., 1.;

  system.Initialize();

}

TEST_F(TestLinearDamping, DampingCoeff) {
  force->SetDampingCoeff(0, 0, damp(0, 0));
  CheckForce(Force(-damp(0, 0) * vx, 0., 0.), Torque());
}

TEST_F(TestLinearDamping, DiagonalTranslationalDamping) {
  force->SetDiagonalTranslationDamping(damp(0, 0), damp(1, 1), damp(2, 2));
  CheckForce(Force(-damp(0, 0) * vx, -damp(1, 1) * vy, -damp(2, 2) * vz), Torque());
}

TEST_F(TestLinearDamping, DiagonalRotationalDamping) {
  force->SetDiagonalRotationDamping(damp(3, 3), damp(4, 4), damp(5, 5));
  CheckForce(Force(), Torque(-damp(3, 3) * vp, -damp(4, 4) * vq, -damp(5, 5) * vr));
}

TEST_F(TestLinearDamping, DiagonalDamping) {
  force->SetDiagonalDamping(damp(0, 0), damp(1, 1), damp(2, 2), damp(3, 3), damp(4, 4), damp(5, 5));
  CheckForce(Force(-damp(0, 0) * vx, -damp(1, 1) * vy, -damp(2, 2) * vz),
             Torque(-damp(3, 3) * vp, -damp(4, 4) * vq, -damp(5, 5) * vr));
}

TEST_F(TestLinearDamping, DampingMatrix) {
  force->SetDampingMatrix(damp);
  auto vs = GeneralizedVelocity(Velocity(vx, vy, vz), AngularVelocity(vp, vq, vr));
  Vector6d<double> res = -damp * vs;
  CheckForce(Force(res(0), res(1), res(2)), Torque(res(3), res(4), res(5)));
}

TEST_F(TestLinearDamping, RelativeVelocity) {

  force->SetDampingMatrix(damp);
  force->SetRelativeToFluid(true);

  Velocity bodyVelocityInWorld = body->GetCOGLinearVelocityInWorld(NWU);
  Velocity relativeVelocityInWorld = bodyVelocityInWorld - flowVelocity;
  Velocity relativeVelocityInBody = body->ProjectVectorInBody(relativeVelocityInWorld, NWU);

  auto vs = GeneralizedVelocity(relativeVelocityInBody, AngularVelocity(vp, vq, vr));
  Vector6d<double> res = -damp * vs;

  CheckForce(Force(res(0), res(1), res(2)), Torque(res(3), res(4), res(5)));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
