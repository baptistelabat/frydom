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

class TestManDampingTaylorExpansion : public ::testing::Test {

protected:
    FrOffshoreSystem system;
    std::shared_ptr<FrBody> body;
    std::shared_ptr<FrManDampingTaylorExpansion> force;

protected:

    void SetUp() override;

public:

    void CheckForce(Force forceRef, Torque torqueRef) const {
        force->Update(0.);
        auto worldForce = force->GetForceInWorld(NWU);
        auto bodyTorque = force->GetTorqueInBodyAtCOG(NWU);
        EXPECT_FLOAT_EQ(forceRef.GetFx(), worldForce.GetFx());
        EXPECT_FLOAT_EQ(forceRef.GetFy(), worldForce.GetFy());
        EXPECT_FLOAT_EQ(forceRef.GetFz(), worldForce.GetFz());
        EXPECT_FLOAT_EQ(torqueRef.GetMx(), bodyTorque.GetMx());
        EXPECT_FLOAT_EQ(torqueRef.GetMy(), bodyTorque.GetMy());
        EXPECT_FLOAT_EQ(torqueRef.GetMz(), bodyTorque.GetMz());
    }

};

void TestManDampingTaylorExpansion::SetUp() {
    body = system.NewBody();
    force = std::make_shared<FrManDampingTaylorExpansion>();
    body->AddExternalForce(force);
}

TEST_F(TestManDampingTaylorExpansion, Xparam) {

    double vx = 1.5;
    double vy = 0.9;
    double vrz = 0.1;

    body->SetGeneralizedVelocityInWorld(Velocity(vx, vy, 0.), AngularVelocity(0., 0., vrz), NWU);

    force->SetX(10., 1, 0, 0);
    CheckForce(Force(-10.*vx, 0., 0.), Torque(0., 0., 0.));

    force->ClearAll();
    force->SetX(10., 2, 0, 0);
    CheckForce(Force(-10.*vx*vx, 0., 0.), Torque(0., 0., 0.));

    force->ClearAll();
    force->SetX(10., 3, 0, 0);
    CheckForce(Force(-10.*vx*vx*vx, 0., 0.), Torque(0., 0., 0.));

    force->ClearAll();
    force->SetX(10., 3, 2, 0);
    CheckForce(Force(-10.*vx*vx*vx*vy*vy, 0., 0.), Torque(0., 0., 0.));

    force->ClearAll();
    force->SetX(10., 3, 2, 1);
    CheckForce(Force(-10.*vx*vx*vx*vy*vy*vrz, 0., 0.), Torque(0., 0., 0.));

    force->ClearAll();
    force->SetX("uuuvvw", 10.);
    CheckForce(Force(-10.*vx*vx*vx*vy*vy*vrz, 0., 0.), Torque(0., 0., 0.));

    force->ClearAll();
    force->Set("Xuuuvvw", 10.);
    CheckForce(Force(-10.*vx*vx*vx*vy*vy*vrz, 0., 0.), Torque(0., 0., 0.));
}

TEST_F(TestManDampingTaylorExpansion, XYparam) {
    double vx = 1.5;
    double vy = 0.9;
    double vrz = 0.1;

    body->SetGeneralizedVelocityInWorld(Velocity(vx, vy, 0.), AngularVelocity(0., 0., vrz), NWU);
    force->SetX(10., 3, 2, 1);
    force->SetY(15., 2, 1, 1);
    CheckForce(Force(-10.*vx*vx*vx*vy*vy*vrz, -15*vx*vx*vy*vrz, 0.), Torque(0., 0., 0.));

    force->ClearAll();
    force->Set("Xuuuvvw", 10.);
    force->Set("Yuuvw", 15.);
    CheckForce(Force(-10.*vx*vx*vx*vy*vy*vrz, -15*vx*vx*vy*vrz, 0.), Torque(0., 0., 0.));
}

TEST_F(TestManDampingTaylorExpansion, XYNparam) {
    double vx = 1.5;
    double vy = 0.9;
    double vrz = 0.1;

    body->SetGeneralizedVelocityInWorld(Velocity(vx, vy, 0.), AngularVelocity(0., 0., vrz), NWU);
    force->SetX(10., 3, 2, 1);
    force->SetY(15., 2, 1, 1);
    force->SetN(16., 0, 0, 1);
    CheckForce(Force(-10.*vx*vx*vx*vy*vy*vrz, -15*vx*vx*vy*vrz, 0.), Torque(0., 0., -16.*vrz));

    force->ClearAll();
    force->Set("Xuuuvvw", 10.);
    force->Set("Yuuvw", 15.);
    force->Set("Nw", 16.);
    CheckForce(Force(-10.*vx*vx*vx*vy*vy*vrz, -15*vx*vx*vy*vrz, 0.), Torque(0., 0., -16.*vrz));
}
