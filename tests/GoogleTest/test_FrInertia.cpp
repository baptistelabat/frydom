//
// Created by camille on 12/12/18.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;




class TestInertia : public ::testing::Test {

protected:

    FrOffshoreSystem_ system;
    std::shared_ptr<FrBody_> body;
    std::shared_ptr<FrInertiaTensor_> inertia;

    Position m_BodyPositionInWorld;
    Direction m_BodyRotationDirection;
    double m_BodyRotationAngle;
    Position m_COG;
    double m_BodyMass;

    Position m_PointInBody;
    Direction m_FrameRotationDirection;
    double m_FrameRotationAngle;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_InertialInFrameAtPoint;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_InertialInBodyAtCOG;


    void SetUp() override;

    void LoadData(std::string filename);

    template <class Vector>
    Vector ReadVector(FrHDF5Reader& reader, std::string field) const;

    void CheckInertiaAtCOG() const;

};

template <class Vector>
Vector TestInertia::ReadVector(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return Vector(value(0), value(1), value(2));
}

void TestInertia::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "/inertia/";

    m_BodyPositionInWorld = ReadVector<Position>(reader, group + "BodyPositionInWorld");
    m_BodyRotationDirection = ReadVector<Direction>(reader, group + "BodyRotationDirection");
    m_BodyRotationAngle = reader.ReadDouble(group + "BodyRotationAngle");
    m_COG = ReadVector<Position>(reader, group + "COG");
    m_PointInBody = ReadVector<Position>(reader, group + "PointInBody");
    m_FrameRotationDirection = ReadVector<Direction>(reader, group + "FrameRotationDirection");
    m_FrameRotationAngle = reader.ReadDouble(group + "FrameRotationAngle");
    m_BodyMass = reader.ReadDouble(group + "BodyMass");
    m_InertialInBodyAtCOG = reader.ReadDoubleArray(group + "InertiaInBodyAtCOG");
    m_InertialInFrameAtPoint = reader.ReadDoubleArray(group + "InertiaInFrameAtPoint");

}

void TestInertia::SetUp() {

    this->LoadData("TNR_database.h5");

    body = system.NewBody();
    body->SetPosition(m_BodyPositionInWorld, NWU);
    body->SetRotation(FrUnitQuaternion_(m_BodyRotationDirection, m_BodyRotationAngle, NWU));

    body->SetInertiaTensor(FrInertiaTensor_(m_BodyMass,m_COG,NWU));
//    body->SetCOG(m_COG, NWU);
//    body->SetMass(m_BodyMass);

    system.Initialize();

}

void TestInertia::CheckInertiaAtCOG() const {
    double Ixx, Iyy, Izz;
    double Ixy, Ixz, Iyz;
    inertia->GetInertiaCoeffs(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);

    EXPECT_NEAR(m_InertialInBodyAtCOG(0, 0), Ixx, 1e-8);
    EXPECT_NEAR(m_InertialInBodyAtCOG(1, 1), Iyy, 1e-8);
    EXPECT_NEAR(m_InertialInBodyAtCOG(2, 2), Izz, 1e-8);
    EXPECT_NEAR(m_InertialInBodyAtCOG(0, 1), Ixy, 1e-8);
    EXPECT_NEAR(m_InertialInBodyAtCOG(0, 2), Ixz, 1e-8);
    EXPECT_NEAR(m_InertialInBodyAtCOG(1, 2), Iyz, 1e-8);
}

TEST_F(TestInertia, InertiaInFrame) {

    auto frame = FrFrame_(m_PointInBody, FrRotation_(m_FrameRotationDirection, m_FrameRotationAngle, NWU), NWU);

    inertia = std::make_shared<FrInertiaTensor_>(m_BodyMass,
       m_InertialInFrameAtPoint(0, 0), m_InertialInFrameAtPoint(1, 1), m_InertialInFrameAtPoint(2, 2),
       m_InertialInFrameAtPoint(0, 1), m_InertialInFrameAtPoint(0, 2), m_InertialInFrameAtPoint(1, 2),
       frame, body->GetCOG(NWU), NWU);

    this->CheckInertiaAtCOG();
}

TEST_F(TestInertia, InertiaAtCOG) {

    inertia = std::make_shared<FrInertiaTensor_>(m_BodyMass,
       m_InertialInBodyAtCOG(0, 0), m_InertialInBodyAtCOG(1, 1), m_InertialInBodyAtCOG(2, 2),
       m_InertialInBodyAtCOG(0, 1), m_InertialInBodyAtCOG(0, 2), m_InertialInBodyAtCOG(1, 2),
       body->GetFrameAtCOG(NWU), NWU);

    this->CheckInertiaAtCOG();
}

TEST_F(TestInertia, BodyInertiaAtCOG) {

    FrInertiaTensor_ InertiaTensor(m_BodyMass,
                                   m_InertialInBodyAtCOG(0, 0), m_InertialInBodyAtCOG(1, 1), m_InertialInBodyAtCOG(2, 2),
                                   m_InertialInBodyAtCOG(0, 1), m_InertialInBodyAtCOG(0, 2), m_InertialInBodyAtCOG(1, 2),
                                   body->GetFrameAtCOG(NWU), NWU);
    body->SetInertiaTensor(InertiaTensor);

    inertia = std::make_shared<FrInertiaTensor_>(body->GetInertiaTensor(NWU));
    this->CheckInertiaAtCOG();
}

TEST_F(TestInertia, BodyInertiaInFrame) {

    auto frame = FrFrame_(m_PointInBody, FrRotation_(m_FrameRotationDirection, m_FrameRotationAngle, NWU), NWU);


    body->SetInertiaTensor(FrInertiaTensor_ (m_BodyMass,
                                             m_InertialInFrameAtPoint(0, 0), m_InertialInFrameAtPoint(1, 1),
                                             m_InertialInFrameAtPoint(2, 2),
                                             m_InertialInFrameAtPoint(0, 1), m_InertialInFrameAtPoint(0, 2),
                                             m_InertialInFrameAtPoint(1, 2),
                                             frame, body->GetCOG(NWU), NWU));

    inertia = std::make_shared<FrInertiaTensor_>(body->GetInertiaTensor(NWU));
    this->CheckInertiaAtCOG();
}

TEST_F(TestInertia, BodyInertia) {

    body->SetInertiaTensor(FrInertiaTensor_(m_BodyMass,
                                            m_InertialInBodyAtCOG(0, 0), m_InertialInBodyAtCOG(1, 1),
                                            m_InertialInBodyAtCOG(2, 2),
                                            m_InertialInBodyAtCOG(0, 1), m_InertialInBodyAtCOG(0, 2),
                                            m_InertialInBodyAtCOG(1, 2),
                                            FrFrame_(body->GetCOG(NWU), FrRotation_(), NWU), NWU));

    inertia = std::make_shared<FrInertiaTensor_>(body->GetInertiaTensor(NWU));
    this->CheckInertiaAtCOG();
}

