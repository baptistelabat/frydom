//
// Created by camille on 04/12/18.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/core/FrMatrix.h"
#include "frydom/hydrodynamics/FrLinearHydrostaticForce.h"

using namespace frydom;
using namespace mathutils;

// ----------------------------------------------------------------
//
// TEST OF THE STIFFNESS MATRIX
//
// ----------------------------------------------------------------

class TestLinearHydrostaticStiffnessMatrix : public FrLinearHydrostaticStiffnessMatrix_
{

public:
    void CheckK33(double val) const { EXPECT_FLOAT_EQ(val, GetK33());}
    void CheckK44(double val) const { EXPECT_FLOAT_EQ(val, GetK44());}
    void CheckK55(double val) const { EXPECT_FLOAT_EQ(val, GetK55());}
    void CheckK34(double val) const { EXPECT_FLOAT_EQ(val, GetK34());}
    void CheckK35(double val) const { EXPECT_FLOAT_EQ(val, GetK35());}
    void CheckK45(double val) const { EXPECT_FLOAT_EQ(val, GetK45());}
    void CheckK43(double val) const { EXPECT_FLOAT_EQ(val, m_data.at(1, 0));}
    void CheckK53(double val) const { EXPECT_FLOAT_EQ(val, m_data.at(2, 0));}
    void CheckK54(double val) const { EXPECT_FLOAT_EQ(val, m_data.at(2, 1));}
};

TEST(TestHydrostatic, StiffnessMatrix ) {

    auto hstStiffMatrix = TestLinearHydrostaticStiffnessMatrix();
    hstStiffMatrix.SetK33(2.1); hstStiffMatrix.CheckK33(2.1);
    hstStiffMatrix.SetK44(1.5); hstStiffMatrix.CheckK44(1.5);
    hstStiffMatrix.SetK55(6.6); hstStiffMatrix.CheckK55(6.6);
    hstStiffMatrix.SetK34(5.4); hstStiffMatrix.CheckK34(5.4); hstStiffMatrix.CheckK43(5.4);
    hstStiffMatrix.SetK35(8.6); hstStiffMatrix.CheckK35(8.6); hstStiffMatrix.CheckK53(8.6);
    hstStiffMatrix.SetK45(2.2); hstStiffMatrix.CheckK45(2.2); hstStiffMatrix.CheckK54(2.2);

    hstStiffMatrix.SetDiagonal(1.2, 5.5, 1.6);
    hstStiffMatrix.CheckK33(1.2);
    hstStiffMatrix.CheckK44(5.5);
    hstStiffMatrix.CheckK55(1.6);

    hstStiffMatrix.SetNonDiagonal(1.5, 5.8, 5.6);
    hstStiffMatrix.CheckK34(1.5); hstStiffMatrix.CheckK43(1.5);
    hstStiffMatrix.CheckK35(5.8); hstStiffMatrix.CheckK53(5.8);
    hstStiffMatrix.CheckK45(5.6); hstStiffMatrix.CheckK54(5.6);

}

// ----------------------------------------------------------------
//
// TEST OF THE HYDROSTATIC FORCE
//
// ----------------------------------------------------------------

class TestLinearHydrostaticForce_ : public testing::Test {

protected:
    FrOffshoreSystem_ system;
    std::shared_ptr<FrBody_> body;
    std::shared_ptr<FrEquilibriumFrame_> eqFrame;
    std::shared_ptr<FrLinearHydrostaticForce_> hstForce;
    mathutils::Matrix33<double> stiffnessMatrix;
    Force ForceInWorld;
    Torque TorqueInBody;

protected:
    void SetUp() override;
    void LoadData(std::string filename);

    /// Vector reader
    template <class Vector>
    Vector ReadVector(FrHDF5Reader& reader, std::string field) const;

public:

};

template <class Vector>
Vector TestLinearHydrostaticForce_::ReadVector(FrHDF5Reader& reader, std::string field) const {
    auto value = reader.ReadDoubleArray(field);
    return Vector(value(0), value(1), value(2));
}

void TestLinearHydrostaticForce_::SetUp() {
    body = std::make_shared<FrBody_>();
    system.AddBody(body);
    LoadData("TNR_database.h5");
    system.Initialize();
}

void TestLinearHydrostaticForce_::LoadData(std::string filename) {

    FrHDF5Reader reader;

    reader.SetFilename(filename);

    auto bodyPosition = ReadVector<Position>(reader, "/body_frame/PointInWorld");
    auto bodyDirection = ReadVector<Direction>(reader, "/body_frame/RotationDirection");
    auto bodyAngle = reader.ReadDouble("/body_frame/RotationAngle");
    auto bodyCOG = ReadVector<Position>(reader, "/body_frame/COG");
    body->SetFrame(FrFrame_(bodyPosition, FrUnitQuaternion_(bodyDirection, bodyAngle, NWU), NWU));

    FrInertiaTensor_ InertiaTensor(1.,bodyCOG,NWU);
    body->SetInertiaTensor(InertiaTensor);

    auto eqPosition = ReadVector<Position>(reader, "/equilibrium_frame/PointInWorld");
    auto eqDirection = ReadVector<Direction>(reader, "/equilibrium_frame/RotationDirection");
    auto eqAngle = reader.ReadDouble("/equilibrium_frame/RotationAngle");
    eqFrame = std::make_shared<FrEquilibriumFrame_>(eqPosition, FrUnitQuaternion_(eqDirection, eqAngle, NWU),
                                                    NWU, body.get());

    hstForce = std::make_shared<FrLinearHydrostaticForce_>(eqFrame);
    stiffnessMatrix = reader.ReadDoubleArray("/hydrostatic/StiffnessMatrix");
    hstForce->GetStiffnessMatrix()->SetData(stiffnessMatrix);
    body->AddExternalForce(hstForce);

    ForceInWorld = ReadVector<Force>(reader, "/hydrostatic/ForceInWorld");
    TorqueInBody = ReadVector<Torque>(reader, "/hydrostatic/TorqueInBody");
}

TEST_F(TestLinearHydrostaticForce_, force) {

    hstForce->Update(0.);
    auto force = hstForce->GetForceInWorld(NWU);
    force.GetFz() += system.GetGravityAcceleration() * body->GetMass();
    auto torque = hstForce->GetTorqueInBodyAtCOG(NWU);

    EXPECT_FLOAT_EQ(ForceInWorld.GetFx(), force.GetFx());
    EXPECT_FLOAT_EQ(ForceInWorld.GetFy(), force.GetFy());
    EXPECT_FLOAT_EQ(ForceInWorld.GetFz(), force.GetFz());

    EXPECT_FLOAT_EQ(TorqueInBody.GetMx(), torque.GetMx());
    EXPECT_FLOAT_EQ(TorqueInBody.GetMy(), torque.GetMy());
    EXPECT_FLOAT_EQ(TorqueInBody.GetMz(), torque.GetMz());
}