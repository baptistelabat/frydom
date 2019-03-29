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

class TestQuadraticDamping : public ::testing::Test {

protected:

    FrOffshoreSystem system;
    std::shared_ptr<FrBody> body;
    std::shared_ptr<FrQuadraticDamping> force;

    Position bodyPosition = Position(150., 3., 0.5);
    Position cogPosition = Position(0.2, 0.2, 0.3);
    Direction rotationDirection = Direction(0.1, 0.2, 0.9);
    double rotationAngle =  M_PI/6.;

    Velocity bodyVelocity = Velocity(0.5, -1.3, 0.1);
    double Cu = 0.5;
    double Cv = 0.6;
    double Cw = 0.8;
    double Su = 120.;
    double Sv = 160.;
    double Sw = 180.;
    Force forceRef = Force(-7702.5, 83310.24, -739.44);

    void SetUp() override;

};

void TestQuadraticDamping::SetUp() {
    body = system.NewBody();
    body->SetFixedInWorld(true);
    force = std::make_shared<FrQuadraticDamping>(WATER, false);
    body->AddExternalForce(force);
    body->SetPosition(bodyPosition, NWU);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,FrFrame(cogPosition,FrRotation(),NWU),NWU);
    body->SetInertiaTensor(InertiaTensor);

    rotationDirection.normalize();
    body->SetRotation(FrUnitQuaternion(rotationDirection, rotationAngle, NWU));
}

TEST_F(TestQuadraticDamping, TestBodyVelocity) {

    force->SetDampingCoefficients(Cu, Cv, Cw);
    force->SetProjectedSections(Su, Sv, Sw);

    system.Initialize();
    body->SetVelocityInBodyNoRotation(bodyVelocity, NWU);
    force->Update(0.);

    auto bodyForce = force->GetForceInBody(NWU);
    EXPECT_NEAR(forceRef.GetFx(), bodyForce.GetFx(), 1e-5);
    EXPECT_NEAR(forceRef.GetFy(), bodyForce.GetFy(), 1e-5);
    EXPECT_NEAR(forceRef.GetFz(), bodyForce.GetFz(), 1e-5);

}

TEST_F(TestQuadraticDamping, TestCurrentVelocity) {

    force->SetDampingCoefficients(Cu, Cv, Cw);
    force->SetProjectedSections(Su, Sv, Sw);

    system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
    auto frameAtCOG = body->GetFrameAtCOG(NWU);
    Velocity flowVelocity = frameAtCOG.ProjectVectorFrameInParent(Velocity(-0.5, 1.3, 0.), NWU);
    system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(flowVelocity, NWU, GOTO);
    body->SetVelocityInBodyNoRotation(Velocity(0., 0., 0.1), NWU);
    force->SetRelative2Fluid(true);
    system.Initialize();

    force->Update(0.);

    auto bodyForce = force->GetForceInBody(NWU);
    EXPECT_NEAR(forceRef.GetFx(), bodyForce.GetFx(), 1e-5);
    EXPECT_NEAR(forceRef.GetFy(), bodyForce.GetFy(), 1e-5);
    EXPECT_NEAR(forceRef.GetFz(), bodyForce.GetFz(), 1e-5);
}

TEST_F(TestQuadraticDamping, TestWindVelocity) {

    auto windDamping = std::make_shared<FrQuadraticDamping>(AIR, true);
    body->AddExternalForce(windDamping);

    windDamping->SetDampingCoefficients(Cu, Cv, Cw);
    windDamping->SetProjectedSections(Su, Sv, Sw);

    system.GetEnvironment()->GetAtmosphere()->GetWind()->MakeFieldUniform();
    auto frameAtCOG = body->GetFrameAtCOG(NWU);
    Velocity flowVelocity = frameAtCOG.ProjectVectorFrameInParent(Velocity(-0.5, 1.3, 0.), NWU);
    system.GetEnvironment()->GetAtmosphere()->GetWind()->GetFieldUniform()->Set(flowVelocity, NWU, GOTO);
    body->SetVelocityInBodyNoRotation(Velocity(0., 0., 0.1), NWU);
    system.Initialize();

    windDamping->Update(0.);

    auto bodyForce = windDamping->GetForceInBody(NWU);
    EXPECT_NEAR(forceRef.GetFx() * 0.001172346640701071, bodyForce.GetFx(), 1e-5);
    EXPECT_NEAR(forceRef.GetFy() * 0.001172346640701071, bodyForce.GetFy(), 1e-5);
    EXPECT_NEAR(forceRef.GetFz() * 0.001172346640701071, bodyForce.GetFz(), 1e-5);

}
