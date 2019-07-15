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

class TestITTCResistance : public ::testing::Test {

protected:

    double LengthBetweenPerpendicular;
    double hullFormFactor;
    double hullWetSurface;
    double frontalArea;
    double m_cr;
    double m_ca;
    double m_ct;
    double m_cf;
    double m_caa;
    double m_capp;
    double speed;
    double reynoldsNumber;
    double lengthAtWaterLine;
    double m_waterDensity;

    FrOffshoreSystem system;
    std::shared_ptr<FrBody> body;
    std::shared_ptr<FrITTCResistance> force;

    void SetUp() override;

    void LoadData(std::string filename);

};

void TestITTCResistance::SetUp() {
    body = system.NewBody();
    body->SetPosition(Position(105., -2., 0.3), NWU);
    auto direction = Direction(0.1, 0.3, 0.9);
    direction.normalize();
    body->SetRotation(FrUnitQuaternion(direction, M_PI/5., NWU));

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,Position(0.2, 0.2, 0.1),NWU);
    body->SetInertiaTensor(InertiaTensor);

    system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
    cppfs::FilePath resources_path(std::string(RESOURCES_PATH));
    LoadData(resources_path.resolve("TNR_database.h5").path());
}

void TestITTCResistance::LoadData(std::string filename) {

    FrHDF5Reader reader;
    std::string group = "/ittc/";

    reader.SetFilename(filename);

    LengthBetweenPerpendicular = reader.ReadDouble(group + "LengthBetweenPerpendicular");
    hullFormFactor = reader.ReadDouble(group + "HullFormParameter");
    hullWetSurface = reader.ReadDouble(group + "WettedSurfaceArea");
    frontalArea = reader.ReadDouble(group + "FrontalArea");
    lengthAtWaterLine = reader.ReadDouble(group + "LengthAtTheWaterLine");
    reynoldsNumber = reader.ReadDouble(group + "ReynoldsNumber");

    speed = reader.ReadDouble(group + "Speed");

    m_ct = reader.ReadDouble(group + "CT");
    m_cr = reader.ReadDouble(group + "CR");
    m_cf = reader.ReadDouble(group + "CF");
    m_ca = reader.ReadDouble(group + "CA");
    m_caa = reader.ReadDouble(group + "CAA");
    m_capp = reader.ReadDouble(group + "CAPP");

    m_waterDensity = reader.ReadDouble(group + "waterDensity");
    system.GetEnvironment()->GetOcean()->SetDensity(m_waterDensity);
    auto kinematicViscosity = reader.ReadDouble(group + "KinematicViscosity");
    system.GetEnvironment()->GetOcean()->SetKinematicViscosity(kinematicViscosity);
}

TEST_F(TestITTCResistance, test0) {
    force = std::make_shared<FrITTCResistance>(LengthBetweenPerpendicular, hullWetSurface, m_cr);
    force->SetHullFormFactor(hullFormFactor);
    force->SetRoughnessFromLength(lengthAtWaterLine);
    force->SetAirResistanceFromArea(frontalArea);
    force->SetAppendageCoefficient(m_capp);

    body->AddExternalForce(force);
    system.Initialize();

    body->SetVelocityInBodyNoRotation(Velocity(speed, 0, 0), NWU);
    force->Update(0.);

    auto ct = force->GetForceInBody(NWU) / (-0.5 * m_waterDensity * speed * speed * hullWetSurface);

    EXPECT_NEAR(m_ct, ct.x(), 1e-8);
    EXPECT_NEAR(0., ct.y(), 1e-8);
    EXPECT_NEAR(0., ct.z(), 1e-8);
}
