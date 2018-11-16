//
// Created by camille on 08/11/18.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"

using namespace frydom;

//
// CLASS OF THE TEST
//


class TestFrForce_ : public FrForce_ {

private:
    Position m_PointCOGInWorld;                 ///< Position of the COG in world
    Position m_PointCOGInBody;                  ///< Position of the COG in body
    Position m_PointREFInWorld;                 ///< Position of the body in world

    FrFrame_ m_frameREF;                        ///< Body frame
    FrUnitQuaternion_ m_quatREF;                    ///< Rotation of the body in world

    Position m_PointInBody;                     ///< Position of a point of the body in body
    Position m_PointInWorld;                    ///< Position of a point of the boyd in world

    Force m_forceInWorldAtCOG;                  ///< Force at the COG in world frame
    Force m_forceInBodyAtCOG;                   ///< Force at the COG in body frame
    Force m_forceInWorldAtPoint;                ///< Force at a point of the body in world frame
    Force m_forceInBodyAtPoint;                 ///< Force at a point of the body in body frame

    Torque m_torqueInWorldAtCOG;                ///< Torque at COG in world frame
    Torque m_torqueInBodyAtCOG;                 ///< Torque at COG in body frame
    Torque m_torqueInWorldAtPoint;              ///< Torque at a point of the body in world frame
    Torque m_torqueInBodyAtPoint;               ///< Torque at a point of the body in body frame

    Position m_PointInBody_NED;                 ///< Position of a point of the body in body (NED)
    Position m_PointInWorld_NED;                ///< Position of a point of the body in world (NED)

    Force m_forceInWorldAtCOG_NED;              ///< Force at the COG in world frame (NED)
    Force m_forceInBodyAtCOG_NED;               ///< Force at the COG in body frame (NED)
    Force m_forceInWorldAtPoint_NED;            ///< Force at a point of the body in world frame (NED)
    Force m_forceInBodyAtPoint_NED;             ///< Force at a point of the body in body frame (NED)

    Torque m_torqueInWorldAtCOG_NED;            ///< Torque at COG in world frame (NED)
    Torque m_torqueInBodyAtCOG_NED;             ///< Torque at COG in body frame (NED)
    Torque m_torqueInWorldAtPoint_NED;          ///< Torque at a point of the body in world frame (NED)
    Torque m_torqueInBodyAtPoint_NED;           ///< Torque at a point of the body in body frame (NED)

    double m_forceLimitUser;
    double m_torqueLimitUser;
    Force m_forceInWorldAtCOG_limited;          ///< Force at the COG in world frame with applied limit
    Torque m_torqueInBodyAtCOG_limited;         ///< Torque at the COG in body frame with appliced limit


public:
    /// Tests
    void TestMaxForceLimit(double fmax);
    void TestMaxTorqueLimit(double fmax);
    void TestSetLimit();
    void TestForceTorqueInWorldAtPointInBody();
    void TestForceTorqueInWorldAtPointInWorld();
    void TestForceTorqueInBodyAtPointInBody();
    void TestForceTorqueInBodyAtPointInWorld();
    void TestForceTorqueInWorldAtCOG();
    void TestForceTorqueInBodyAtCOG();
    void TestTorqueInBodyAtCOG();
    void TestTorqueInWorldAtCOG();
    void TestForceInWorldAtCOG();
    void TestForceInBodyAtCOG();
    void TestForceInWorldAtPointInBody();
    void TestForceInWorldAtPointInWorld();
    void TestForceInBodyAtPointInWorld();
    void TestForceInBodyAtPointInBody();
    void TestGetForceInWorldReference();
    void TestGetForceInWorldComponent();
    void TestGetForceInBody();
    void TestGetForceInBodyReference();
    void TestGetForceInBodyComponent();
    void TestGetTorqueInWorld();
    void TestGetTorqueInWorldReference();
    void TestGetTorqueInWorldComponent();
    void TestGetTorqueInBodyReference();
    void TestGetTorqueInBodyComponent();
    void TestForceNorm();
    void TestTorqueNorm();
    void TestGetForceNED();
    void TestGetTorqueNED();
    void TestSpecificCase();
    void TestUpdateLimitApplication();

    /// Accessor
    Position GetPointREFInWorld() const { return m_PointREFInWorld; }
    Position GetPointCOGInBody()  const { return m_PointCOGInBody; }
    FrUnitQuaternion_ GetQuatREF()    const { return m_quatREF; }

    /// Methods
    void LoadData(std::string filename);
    Position ReadPosition(FrHDF5Reader& reader, std::string field);
    Force ReadForce(FrHDF5Reader& reader, std::string field);
    Torque ReadTorque(FrHDF5Reader& reader, std::string field);
    Direction ReadDirection(FrHDF5Reader& reader, std::string field);

    void CheckForceInWorldAtCOG();
    void CheckTorqueInBodyAtCOG();
    void CheckForceTorqueAtCOG();
    void CheckLeverArmTorqueInBodyAtCOG();

private:
    /// Override pure virtual methods
    void Update(double time) override {};
    void Initialize() override {};
    void StepFinalize() override {};
};

Position TestFrForce_::ReadPosition(FrHDF5Reader& reader, std::string field) {
    auto value = reader.ReadDoubleArray(field);
    return Position(value(0), value(1), value(2));
}

Force TestFrForce_::ReadForce(FrHDF5Reader& reader, std::string field) {
    auto value = reader.ReadDoubleArray(field);
    return Force(value(0), value(1), value(2));
}

Torque TestFrForce_::ReadTorque(FrHDF5Reader& reader, std::string field) {
    auto value = reader.ReadDoubleArray(field);
    return Torque(value(0), value(1), value(2));
}

Direction TestFrForce_::ReadDirection(FrHDF5Reader& reader, std::string field) {
    auto value = reader.ReadDoubleArray(field);
    return Torque(value(0), value(1), value(2));
}

void TestFrForce_::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);
    std::string group = "/force_general/";

    m_PointREFInWorld = ReadPosition(reader, group + "PointREFInWorld/");
    m_PointCOGInBody  = ReadPosition(reader, group + "PointCOGInBody/");
    m_PointInBody     = ReadPosition(reader, group + "PointInBody/");
    m_PointCOGInWorld = ReadPosition(reader, group + "PointCOGInWorld/");
    m_PointInWorld    = ReadPosition(reader, group + "PointInWorld/");

    auto direction = ReadDirection(reader, group + "RotationDirection/");
    direction.normalize();
    auto angle = reader.ReadDouble(group + "RotationAngle/");
    m_quatREF =  FrUnitQuaternion_(direction, angle, NWU);
    m_frameREF = FrFrame_(m_PointREFInWorld, m_quatREF, NWU);

    m_forceInWorldAtPoint  = ReadForce(reader, group + "ForceInWorldAtPoint/");
    m_forceInBodyAtPoint   = ReadForce(reader, group + "ForceInBodyAtPoint/");
    m_forceInWorldAtCOG    = ReadForce(reader, group + "ForceInWorldAtCOG/");
    m_forceInBodyAtCOG     = ReadForce(reader, group + "ForceInBodyAtCOG/");
    m_torqueInWorldAtPoint = ReadTorque(reader, group + "TorqueInWorldAtPoint/");
    m_torqueInBodyAtPoint  = ReadTorque(reader, group + "TorqueInBodyAtPoint/");
    m_torqueInWorldAtCOG   = ReadTorque(reader, group + "TorqueInWorldAtCOG/");
    m_torqueInBodyAtCOG    = ReadTorque(reader, group + "TorqueInBodyAtCOG/");

    // Create vector in NED convention
    m_PointInBody_NED          = m_PointInBody;
    m_PointInWorld_NED         = m_PointInWorld;
    m_forceInWorldAtPoint_NED  = m_forceInWorldAtPoint;
    m_forceInBodyAtPoint_NED   = m_forceInBodyAtPoint;
    m_forceInWorldAtCOG_NED    = m_forceInWorldAtCOG;
    m_forceInBodyAtCOG_NED     = m_forceInBodyAtCOG;
    m_torqueInWorldAtPoint_NED = m_torqueInWorldAtPoint;
    m_torqueInBodyAtPoint_NED  = m_torqueInBodyAtPoint;
    m_torqueInWorldAtCOG_NED   = m_torqueInWorldAtCOG;
    m_torqueInBodyAtCOG_NED    = m_torqueInBodyAtCOG;
    internal::SwapFrameConvention(m_PointInBody_NED);
    internal::SwapFrameConvention(m_PointInWorld_NED);
    internal::SwapFrameConvention(m_forceInWorldAtPoint_NED);
    internal::SwapFrameConvention(m_forceInBodyAtPoint_NED);
    internal::SwapFrameConvention(m_forceInWorldAtCOG_NED);
    internal::SwapFrameConvention(m_forceInBodyAtCOG_NED);
    internal::SwapFrameConvention(m_torqueInWorldAtPoint_NED);
    internal::SwapFrameConvention(m_torqueInBodyAtPoint_NED);
    internal::SwapFrameConvention(m_torqueInWorldAtCOG_NED);
    internal::SwapFrameConvention(m_torqueInBodyAtCOG_NED);

    m_forceLimitUser       = reader.ReadDouble(group + "ForceLimit/");
    m_torqueLimitUser      = reader.ReadDouble(group + "TorqueLimit/");
    m_forceInWorldAtCOG_limited = ReadForce(reader, group + "ForceInWorldAtCOG_limited/");
    m_torqueInBodyAtCOG_limited = ReadTorque(reader, group + "TorqueInBodyAtCOG_limited/");
}

void TestFrForce_::CheckForceInWorldAtCOG() {
    m_body->Update();
    auto force = this->GetForceInWorld(NWU);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());
}

void TestFrForce_::CheckTorqueInBodyAtCOG() {
    m_body->Update();
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());
}

void TestFrForce_::CheckForceTorqueAtCOG() {
    this->CheckForceInWorldAtCOG();
    this->CheckTorqueInBodyAtCOG();
}

void TestFrForce_::CheckLeverArmTorqueInBodyAtCOG() {
    m_body->Update();
    auto torque = this->GetTorqueInBodyAtCOG(NWU);
    Torque torqueREF = m_torqueInBodyAtCOG - m_torqueInBodyAtPoint;

    EXPECT_FLOAT_EQ(torque.GetMx(), torqueREF.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), torqueREF.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), torqueREF.GetMz());
}


void TestFrForce_::TestMaxForceLimit(double fmax) {
    this->SetMaxForceLimit(fmax);
    EXPECT_FLOAT_EQ(this->GetMaxForceLimit(), fmax);
}

void TestFrForce_::TestMaxTorqueLimit(double fmax) {
    this->SetMaxTorqueLimit(fmax);
    EXPECT_FLOAT_EQ(this->GetMaxTorqueLimit(), fmax);
}

void TestFrForce_::TestSetLimit() {
    this->SetLimit(true);
    EXPECT_TRUE(this->GetLimit());
}

void TestFrForce_::TestForceTorqueInWorldAtPointInBody() {
    this->SetForceTorqueInWorldAtPointInBody(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInBody, NWU);
    CheckForceTorqueAtCOG();
    this->SetForceTorqueInWorldAtPointInBody(m_forceInWorldAtPoint_NED, m_torqueInWorldAtPoint_NED, m_PointInBody_NED, NED);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestForceTorqueInBodyAtPointInBody() {
    this->SetForceTorqueInBodyAtPointInBody(m_forceInBodyAtPoint, m_torqueInBodyAtPoint, m_PointInBody, NWU);
    CheckForceTorqueAtCOG();
    this->SetForceTorqueInBodyAtPointInBody(m_forceInBodyAtPoint_NED, m_torqueInBodyAtPoint_NED, m_PointInBody_NED, NED);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestForceTorqueInWorldAtPointInWorld() {
    this->SetForceTorqueInWorldAtPointInWorld(m_forceInWorldAtPoint, m_torqueInWorldAtPoint, m_PointInWorld, NWU);
    CheckForceTorqueAtCOG();
    this->SetForceTorqueInWorldAtPointInWorld(m_forceInWorldAtPoint_NED, m_torqueInWorldAtPoint_NED, m_PointInWorld_NED, NED);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestForceTorqueInBodyAtPointInWorld() {
    this->SetForceTorqueInBodyAtPointInWorld(m_forceInBodyAtPoint, m_torqueInBodyAtPoint, m_PointInWorld, NWU);
    CheckForceTorqueAtCOG();
    this->SetForceTorqueInBodyAtPointInWorld(m_forceInBodyAtPoint_NED, m_torqueInBodyAtPoint_NED, m_PointInWorld_NED, NED);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestForceTorqueInWorldAtCOG() {
    this->SetForceTorqueInWorldAtCOG(m_forceInWorldAtCOG, m_torqueInWorldAtCOG, NWU);
    CheckForceTorqueAtCOG();
    this->SetForceTorqueInWorldAtCOG(m_forceInWorldAtCOG_NED, m_torqueInWorldAtCOG_NED, NED);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestForceTorqueInBodyAtCOG() {
    this->SetForceTorqueInBodyAtCOG(m_forceInBodyAtCOG, m_torqueInBodyAtCOG, NWU);
    CheckForceTorqueAtCOG();
    this->SetForceTorqueInBodyAtCOG(m_forceInBodyAtCOG_NED, m_torqueInBodyAtCOG_NED, NED);
    CheckForceTorqueAtCOG();
}

void TestFrForce_::TestTorqueInBodyAtCOG() {
    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    CheckTorqueInBodyAtCOG();
    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG_NED, NED);
    CheckTorqueInBodyAtCOG();
}

void TestFrForce_::TestTorqueInWorldAtCOG() {
    this->SetTorqueInWorldAtCOG(m_torqueInWorldAtCOG, NWU);
    CheckTorqueInBodyAtCOG();
    this->SetTorqueInWorldAtCOG(m_torqueInWorldAtCOG_NED, NED);
    CheckTorqueInBodyAtCOG();
}

void TestFrForce_::TestForceInBodyAtCOG() {
    this->SetForceInBody(m_forceInBodyAtCOG, NWU);
    CheckForceInWorldAtCOG();
    this->SetForceInBody(m_forceInBodyAtCOG_NED, NED);
    CheckForceInWorldAtCOG();}

void TestFrForce_::TestForceInWorldAtCOG() {
    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);
    CheckForceInWorldAtCOG();
    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG_NED, NED);
    CheckForceInWorldAtCOG();
}

void TestFrForce_::TestForceInWorldAtPointInBody() {
    this->SetForceInWorldAtPointInBody(m_forceInWorldAtPoint, m_PointInBody, NWU);
    CheckForceInWorldAtCOG();
    CheckLeverArmTorqueInBodyAtCOG();
    this->SetForceInWorldAtPointInBody(m_forceInWorldAtPoint_NED, m_PointInBody_NED, NED);
    CheckForceInWorldAtCOG();
    CheckLeverArmTorqueInBodyAtCOG();
}

void TestFrForce_::TestForceInWorldAtPointInWorld() {
    this->SetForceInWorldAtPointInWorld(m_forceInWorldAtPoint, m_PointInWorld, NWU);
    CheckForceInWorldAtCOG();
    CheckLeverArmTorqueInBodyAtCOG();
    this->SetForceInWorldAtPointInWorld(m_forceInWorldAtPoint_NED, m_PointInWorld_NED, NED);
    CheckForceInWorldAtCOG();
    CheckLeverArmTorqueInBodyAtCOG();
}

void TestFrForce_::TestForceInBodyAtPointInBody() {
    this->SetForceInBodyAtPointInBody(m_forceInBodyAtPoint, m_PointInBody, NWU);
    CheckForceInWorldAtCOG();
    CheckLeverArmTorqueInBodyAtCOG();
    this->SetForceInBodyAtPointInBody(m_forceInBodyAtPoint_NED, m_PointInBody_NED, NED);
    CheckForceInWorldAtCOG();
    CheckLeverArmTorqueInBodyAtCOG();
}

void TestFrForce_::TestForceInBodyAtPointInWorld() {
    this->SetForceInBodyAtPointInWorld(m_forceInBodyAtPoint, m_PointInWorld, NWU);
    CheckForceInWorldAtCOG();
    CheckLeverArmTorqueInBodyAtCOG();
    this->SetForceInBodyAtPointInWorld(m_forceInBodyAtPoint_NED, m_PointInWorld_NED, NED);
    CheckForceInWorldAtCOG();
    CheckLeverArmTorqueInBodyAtCOG();
}

void TestFrForce_::TestGetForceInWorldReference() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInWorld(force, NWU);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    GetForceInWorld(force, NED);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG_NED.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG_NED.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG_NED.GetFz());
}

void TestFrForce_::TestGetForceInWorldComponent() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInWorld(force.GetFx(), force.GetFy(), force.GetFz(), NWU);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG.GetFz());

    GetForceInWorld(force.GetFx(), force.GetFy(), force.GetFz(), NED);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG_NED.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG_NED.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG_NED.GetFz());
}

void TestFrForce_::TestGetForceInBody() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    auto force = GetForceInBody(NWU);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

    force = GetForceInBody(NED);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG_NED.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG_NED.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG_NED.GetFz());
}

void TestFrForce_::TestGetForceInBodyReference() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    m_body->Update();

    Force force;
    GetForceInBody(force, NWU);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

    GetForceInBody(force, NED);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG_NED.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG_NED.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG_NED.GetFz());
}


void TestFrForce_::TestGetForceInBodyComponent() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);
    m_body->Update();

    Force force;
    GetForceInBody(force.GetFx(), force.GetFy(), force.GetFz(), NWU);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG.GetFz());

    GetForceInBody(force.GetFx(), force.GetFy(), force.GetFz(), NED);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInBodyAtCOG_NED.GetFx());
    EXPECT_FLOAT_EQ(force.GetFy(), m_forceInBodyAtCOG_NED.GetFy());
    EXPECT_FLOAT_EQ(force.GetFz(), m_forceInBodyAtCOG_NED.GetFz());
}


void TestFrForce_::TestGetTorqueInWorld() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    auto torque = GetTorqueInWorldAtCOG(NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG.GetMz());

    torque = GetTorqueInWorldAtCOG(NED);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG_NED.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG_NED.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG_NED.GetMz());
}

void TestFrForce_::TestGetTorqueInWorldReference() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    Torque torque;
    GetTorqueInWorldAtCOG(torque, NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG.GetMz());

    GetTorqueInWorldAtCOG(torque, NED);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG_NED.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG_NED.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG_NED.GetMz());
}

void TestFrForce_::TestGetTorqueInWorldComponent() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    Torque torque;
    GetTorqueInWorldAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG.GetMz());

    GetTorqueInWorldAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NED);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInWorldAtCOG_NED.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInWorldAtCOG_NED.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInWorldAtCOG_NED.GetMz());
}

void TestFrForce_::TestGetTorqueInBodyReference() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    Torque torque;
    GetTorqueInBodyAtCOG(torque, NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    GetTorqueInBodyAtCOG(torque, NED);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG_NED.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG_NED.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG_NED.GetMz());
}

void TestFrForce_::TestGetTorqueInBodyComponent() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    Torque torque;
    GetTorqueInBodyAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NWU);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG.GetMz());

    GetTorqueInBodyAtCOG(torque.GetMx(), torque.GetMy(), torque.GetMz(), NED);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG_NED.GetMx());
    EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG_NED.GetMy());
    EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG_NED.GetMz());
}

void TestFrForce_::TestForceNorm() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);
    m_body->Update();

    double normREF =  std::sqrt(m_forceInWorldAtCOG.GetFx() * m_forceInWorldAtCOG.GetFx()
                    + m_forceInWorldAtCOG.GetFy() * m_forceInWorldAtCOG.GetFy()
                    + m_forceInWorldAtCOG.GetFz() * m_forceInWorldAtCOG.GetFz());

    auto normTEST = GetForceNorm();

    EXPECT_FLOAT_EQ(normTEST, normREF);
}


void TestFrForce_::TestTorqueNorm() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);
    m_body->Update();

    double normREF =  std::sqrt(m_torqueInBodyAtCOG.GetMx() * m_torqueInBodyAtCOG.GetMx()
                    + m_torqueInBodyAtCOG.GetMy() * m_torqueInBodyAtCOG.GetMy()
                    + m_torqueInBodyAtCOG.GetMz() * m_torqueInBodyAtCOG.GetMz());

    auto normTEST = GetTorqueNormAtCOG();

    EXPECT_FLOAT_EQ(normTEST, normREF);
}

void TestFrForce_::TestGetForceNED() {

    this->SetForceInWorldAtCOG(m_forceInWorldAtCOG, NWU);

    auto force = GetForceInWorld(NED);
    EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG.GetFx());
    EXPECT_FLOAT_EQ(-force.GetFy(), m_forceInWorldAtCOG.GetFy());
    EXPECT_FLOAT_EQ(-force.GetFz(), m_forceInWorldAtCOG.GetFz());
}

void TestFrForce_::TestGetTorqueNED() {

    this->SetTorqueInBodyAtCOG(m_torqueInBodyAtCOG, NWU);

    auto torque = GetTorqueInBodyAtCOG(NED);
    EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG.GetMx());
    EXPECT_FLOAT_EQ(-torque.GetMy(), m_torqueInBodyAtCOG.GetMy());
    EXPECT_FLOAT_EQ(-torque.GetMz(), m_torqueInBodyAtCOG.GetMz());
}

void TestFrForce_::TestSpecificCase() {
    this->SetForceTorqueInBodyAtPointInBody(m_forceInBodyAtCOG, m_torqueInBodyAtCOG, m_PointCOGInBody, NWU);
    CheckForceInWorldAtCOG();
    CheckTorqueInBodyAtCOG();
    this->SetForceTorqueInWorldAtPointInWorld(m_forceInWorldAtCOG, m_torqueInWorldAtCOG, m_PointCOGInWorld, NWU);
    CheckForceInWorldAtCOG();
    CheckTorqueInBodyAtCOG();
}

void TestFrForce_::TestUpdateLimitApplication() {

        this->SetMaxForceLimit(m_forceLimitUser);
        this->SetMaxTorqueLimit(m_torqueLimitUser);
        this->SetLimit(true);

        this->SetForceTorqueInBodyAtCOG(m_forceInBodyAtCOG, m_torqueInBodyAtCOG, NWU);
        m_chronoForce->UpdateState();

        auto force = GetForceInWorld(NWU);
        auto torque = GetTorqueInBodyAtCOG(NWU);

        EXPECT_FLOAT_EQ(force.GetFx(), m_forceInWorldAtCOG_limited.GetFx());
        EXPECT_FLOAT_EQ(force.GetFy(), m_forceInWorldAtCOG_limited.GetFy());
        EXPECT_FLOAT_EQ(force.GetFz(), m_forceInWorldAtCOG_limited.GetFz());

        EXPECT_FLOAT_EQ(torque.GetMx(), m_torqueInBodyAtCOG_limited.GetMx());
        EXPECT_FLOAT_EQ(torque.GetMy(), m_torqueInBodyAtCOG_limited.GetMy());
        EXPECT_FLOAT_EQ(torque.GetMz(), m_torqueInBodyAtCOG_limited.GetMz());
};

//
// TESTING CLASS FUNCTION
//

class TestBase : public ::testing::Test {

protected:

    /// Environment objects
    FrOffshoreSystem_ system;
    std::shared_ptr<FrBody_> body;
    std::shared_ptr<TestFrForce_> force;

    /// Initialize environment
    void SetUp() override;

    /// Create a new body object with position and orientation of the test
    std::shared_ptr<FrBody_> NewBody(std::shared_ptr<TestFrForce_> test);

};

void TestBase::SetUp() {
    force = std::make_shared<TestFrForce_>();
    force->LoadData("TNR_database.h5");
    body = NewBody(force);
    system.AddBody(body);
}

std::shared_ptr<FrBody_> TestBase::NewBody(std::shared_ptr<TestFrForce_> test) {

    auto body = std::make_shared<FrBody_>();
    body->SetPosition(test->GetPointREFInWorld(), NWU);
    body->SetCOG(test->GetPointCOGInBody(), NWU);
    body->SetRotation(test->GetQuatREF());
    body->AddExternalForce(test);
    body->Update();
    return body;
}


//
// LIST OF GTEST
//

TEST(FrForce_test, MaxForceLimit) {
    TestFrForce_ test;
    test.TestMaxForceLimit(5.e6);
}

TEST(FrForce_test, MaxTorqueLimit) {
    TestFrForce_ test;
    test.TestMaxTorqueLimit(9.e9);
}

TEST(FrForce_test, SetLimit) {
    TestFrForce_ test;
    test.TestSetLimit();
}

TEST_F(TestBase, ForceTorqueInWorldAtPointInBody) {
    force->TestForceTorqueInWorldAtPointInBody();
}

TEST_F(TestBase, ForceTorqueInBodyAtPointInBody) {
    force->TestForceTorqueInBodyAtPointInBody();
}

TEST_F(TestBase, ForceTorqueInWorldAtPointInWorld) {
    force->TestForceTorqueInWorldAtPointInWorld();
}

TEST_F(TestBase, ForceTorqueInBodyAtPointInWorld) {
    force->TestForceTorqueInBodyAtPointInWorld();
}

TEST_F(TestBase, ForceTorqueInWorldAtCOG) {
    force->TestForceTorqueInWorldAtCOG();
}

TEST_F(TestBase, ForceTorqueInBodyAtCOG) {
    force->TestForceTorqueInBodyAtCOG();
}

TEST_F(TestBase, TorqueInBodyAtCOG) {
    force->TestTorqueInBodyAtCOG();
}

TEST_F(TestBase, TorqueInWorldAtCOG) {
    force->TestTorqueInWorldAtCOG();
}

TEST_F(TestBase, ForceInBodyAtCOG) {
    force->TestForceInBodyAtCOG();
}

TEST_F(TestBase, ForceInWorldAtCOG) {
    force->TestForceInWorldAtCOG();
}

TEST_F(TestBase, ForceInWorldAtPointInBody) {
    force->TestForceInWorldAtPointInBody();
}

TEST_F(TestBase, ForceInWorldAtPointInWorld) {
    force->TestForceInWorldAtPointInWorld();
}

TEST_F(TestBase, ForceInBodyAtPointInBody) {
    force->TestForceInBodyAtPointInBody();
}

TEST_F(TestBase, ForceInBodyAtPointInWorld) {
    force->TestForceInBodyAtPointInWorld();
}

TEST_F(TestBase, GetForceInWorldReference) {
   force->TestGetForceInWorldReference();
}

TEST_F(TestBase, GetForceInWorldComponent) {
    force->TestGetForceInWorldComponent();
}

TEST_F(TestBase, GetForceInBody) {
    force->TestGetForceInBody();
}

TEST_F(TestBase, GetForceInBodyReference) {
    force->TestGetForceInBodyReference();
}

TEST_F(TestBase, GetForceInBodyComponent) {
    force->TestGetForceInBodyComponent();
}

TEST_F(TestBase, GetTorqueInWorld) {
    force->TestGetTorqueInWorld();
}

TEST_F(TestBase, GetTorqueInWorldReference) {
    force->TestGetTorqueInWorldReference();
}

TEST_F(TestBase, GetTorqueInBodyReference) {
    force->TestGetTorqueInBodyReference();
}

TEST_F(TestBase,GetTorqueInBodyComponent) {
    force->TestGetTorqueInBodyComponent();
}

TEST_F(TestBase,GetTorqueInWorldComponent) {
    force->TestGetTorqueInWorldComponent();
}

TEST_F(TestBase, ForceNorm) {
    force->TestForceNorm();
}

TEST_F(TestBase, TorqueNorm) {
    force->TestTorqueNorm();
}

TEST_F(TestBase, GetForceNED) {
    force->TestGetForceNED();
}

TEST_F(TestBase, GetTorqueNED) {
    force->TestGetTorqueNED();
}

TEST_F(TestBase, SpecificCase) {
    force->TestSpecificCase();
}

TEST_F(TestBase, UpdateLimitApplication) {
    force->TestUpdateLimitApplication();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
