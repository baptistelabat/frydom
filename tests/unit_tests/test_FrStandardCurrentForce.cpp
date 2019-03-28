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

class TestFrStandardCurrentForce : public ::testing::Test {

protected:

    FrOffshoreSystem system;
    std::shared_ptr<FrBody> body;
    std::shared_ptr<FrCurrentStandardForce> force;

    double m_frontalArea;
    double m_lateralArea;
    double m_lengthBetweenPerpendicular;
    double m_lengthOverAll;
    double m_Xcenter;
    double m_currentSpeed;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_direction;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_dragCoefficient;

    double m_xadim;
    double m_yadim;
    double m_nadim;

    void SetUp() override;

    void LoadData(std::string filename);


};

void TestFrStandardCurrentForce::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "/standardDNV/current/";

    m_frontalArea = reader.ReadDouble(group + "FrontalArea");
    m_lateralArea = reader.ReadDouble(group + "LateralArea");
    m_lengthBetweenPerpendicular = reader.ReadDouble(group + "LengthBetweenPerpendicular");
    m_lengthOverAll = reader.ReadDouble(group + "LengthOverAll");
    m_Xcenter = reader.ReadDouble(group + "Xcenter");
    m_currentSpeed = reader.ReadDouble(group + "CurrentSpeed");
    m_direction = reader.ReadDoubleArray(group + "CurrentDirection");
    m_dragCoefficient = reader.ReadDoubleArray(group + "DragCoefficient");

}

void TestFrStandardCurrentForce::SetUp() {

    LoadData("TNR_database.h5");

    force = std::make_shared<FrCurrentStandardForce>();
    force->SetLengthBetweenPerpendicular(m_lengthBetweenPerpendicular);
    force->SetLateralArea(m_lateralArea);
    force->SetTransverseArea(m_frontalArea);
    force->SetXCenter(m_Xcenter);

    body = system.NewBody();

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,FrFrame(),NWU);
    body->SetInertiaTensor(InertiaTensor);

    body->AddExternalForce(force);

    system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();

    auto rho = system.GetEnvironment()->GetOcean()->GetDensity();
    m_xadim = 0.5*rho*m_currentSpeed*m_currentSpeed*m_frontalArea;
    m_yadim = 0.5*rho*m_currentSpeed*m_currentSpeed*m_lateralArea;
    m_nadim = 0.5*rho*m_currentSpeed*m_currentSpeed*m_lateralArea*m_lengthOverAll;

    system.Initialize();
}

TEST_F(TestFrStandardCurrentForce, TestForce) {

    for (int i=0; i<m_direction.size(); i++) {

        system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()
                ->Set(m_direction(i), m_currentSpeed, DEG, MS, NED, COMEFROM);

        force->Compute(0.);

        EXPECT_NEAR(m_dragCoefficient(0, i), force->GetForceInWorld(NWU).GetFx() / m_xadim, 1.e-5);
        EXPECT_NEAR(m_dragCoefficient(1, i), force->GetForceInWorld(NWU).GetFy() / m_yadim, 1.e-5);
        EXPECT_NEAR(m_dragCoefficient(2, i), force->GetTorqueInWorldAtCOG(NWU).GetMz() / m_nadim, 1.e-5);
    }
}

TEST_F(TestFrStandardCurrentForce, TestTransport) {

    int i = 2;
    double xc = 0.5;

    system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()
            ->Set(m_direction(i), m_currentSpeed, DEG, MS, NED, COMEFROM);

    FrInertiaTensor InertiaTensor(1.,1.,1.,1.,0.,0.,0.,FrFrame(Position(0.1, 0., 0.),FrRotation(),NWU),NWU);
    body->SetInertiaTensor(InertiaTensor);

    body->Initialize();

    force->SetXCenter(xc);
    force->Initialize();
    force->Compute(0.);

    double torqueRef = m_dragCoefficient(2, i) * m_nadim + (xc-0.1) * m_dragCoefficient(1, i) * m_yadim;
    EXPECT_NEAR(torqueRef / m_nadim, force->GetTorqueInWorldAtCOG(NWU).GetMz() / m_nadim, 1.e-5);
}
