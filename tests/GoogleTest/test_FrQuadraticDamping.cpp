//
// Created by camille on 19/12/18.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;

class TestQuadraticDamping : public ::testing::Test {

protected:

    FrOffshoreSystem_ system;
    std::shared_ptr<FrBody_> body;
    std::shared_ptr<FrQuadraticDamping_> force;

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
    force = std::make_shared<FrQuadraticDamping_>(WATER, false);
    body->AddExternalForce(force);
    body->SetPosition(bodyPosition, NWU);
    body->SetCOG(cogPosition, NWU);
    rotationDirection.normalize();
    body->SetRotation(FrUnitQuaternion_(rotationDirection, rotationAngle, NWU));
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
    Velocity flowVelocity = frameAtCOG.ProjectVectorInParent(Velocity(-0.5, 1.3, 0.));
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

    auto windDamping = std::make_shared<FrQuadraticDamping_>(AIR, true);
    body->AddExternalForce(windDamping);

    windDamping->SetDampingCoefficients(Cu, Cv, Cw);
    windDamping->SetProjectedSections(Su, Sv, Sw);

    system.GetEnvironment()->GetAtmosphere()->GetWind()->MakeFieldUniform();
    auto frameAtCOG = body->GetFrameAtCOG(NWU);
    Velocity flowVelocity = frameAtCOG.ProjectVectorInParent(Velocity(-0.5, 1.3, 0.));
    system.GetEnvironment()->GetAtmosphere()->GetWind()->GetFieldUniform()->Set(flowVelocity, NWU, GOTO);
    body->SetVelocityInBodyNoRotation(Velocity(0., 0., 0.1), NWU);
    system.Initialize();

    windDamping->Update(0.);

    auto bodyForce = windDamping->GetForceInBody(NWU);
    EXPECT_NEAR(forceRef.GetFx() * 0.001172346640701071, bodyForce.GetFx(), 1e-5);
    EXPECT_NEAR(forceRef.GetFy() * 0.001172346640701071, bodyForce.GetFy(), 1e-5);
    EXPECT_NEAR(forceRef.GetFz() * 0.001172346640701071, bodyForce.GetFz(), 1e-5);

}