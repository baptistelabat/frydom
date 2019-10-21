//
// Created by lletourn on 18/10/19.
//


#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;

TEST(FrTorsor, FrTorsor_FrTorsor_Test) {

  FRAME_CONVENTION fc = NWU;

  // Torsor
  mathutils::Vector3d<double> resultante;
  resultante.setRandom();
  mathutils::Vector3d<double> moment;
  moment.setRandom();

  Position point;
  point.setRandom();

  Torsor torsor(resultante, moment, point, fc);

  Position testPos;
  testPos = point - torsor.GetPoint(fc);
  EXPECT_TRUE(testPos.isZero());
}

TEST(FrTorsor, FrTorsor_GeneralizedForce_Test) {

  FRAME_CONVENTION fc = NWU;

  Position point;
  point.setRandom();

  // Generalized Force
  Force force;
  force.setRandom();
  Torque torque;
  torque.setRandom();
  GeneralizedForceTorsor forceTorsor(force, torque, point, fc);

  Position newPoint;
  newPoint.setRandom();
  Torque torqueAtNewCOG = torque + (point - newPoint).cross(force);

  Torque testTorque = torqueAtNewCOG - forceTorsor.GetTorqueAtPoint(newPoint, fc);
  EXPECT_TRUE(testTorque.isZero());

}

TEST(FrTorsor, FrTorsor_GeneralizedVelocity_Test) {

  FRAME_CONVENTION fc = NWU;

  Position point;
  point.setRandom();

  // Generalized Velocity
  Velocity velocity; velocity.setRandom(); AngularVelocity angularVelocity; angularVelocity.setRandom();
  GeneralizedVelocityTorsor velocityTorsor(velocity, angularVelocity, point, fc);

  Position newPoint; newPoint.setRandom();
  Velocity VelocityAtNewCOG = velocity + (point-newPoint).cross(angularVelocity);

  Velocity testVelocity = VelocityAtNewCOG - velocityTorsor.GetLinearVelocityAtPoint(newPoint, fc);
  EXPECT_TRUE(testVelocity.isZero());

}


