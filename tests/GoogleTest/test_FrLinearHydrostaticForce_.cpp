//
// Created by camille on 04/12/18.
//

#include "frydom/frydom.h"
#include "gtest/gtest.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

using namespace frydom;

// ----------------------------------------------------------------
//
// TEST OF THE STIFFNESS MATRIX
//
// ----------------------------------------------------------------

class TestLinearHydrostaticStiffnessMatrix : public FrLinearHydrostaticStiffnessMatrix_
{

public:
    void CheckK33(double val) const { EXPECT_FLOAT_EQ(val, this->at(0, 0));}
    void CheckK44(double val) const { EXPECT_FLOAT_EQ(val, this->at(1, 1));}
    void CheckK55(double val) const { EXPECT_FLOAT_EQ(val, this->at(2, 2));}
    void CheckK34(double val) const { EXPECT_FLOAT_EQ(val, this->at(0, 1));}
    void CheckK35(double val) const { EXPECT_FLOAT_EQ(val, this->at(0, 2));}
    void CheckK45(double val) const { EXPECT_FLOAT_EQ(val, this->at(1, 2));}
    void CheckK43(double val) const { EXPECT_FLOAT_EQ(val, this->at(1, 0));}
    void CheckK53(double val) const { EXPECT_FLOAT_EQ(val, this->at(2, 0));}
    void CheckK54(double val) const { EXPECT_FLOAT_EQ(val, this->at(2, 1));}
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

class TestLinearHydrostaticForce_ : public FrLinearHydrostaticForce_,
                                    public testing::Test {

protected:
    FrOffshoreSystem_ system;
    std::shared_ptr<FrBody_> body;
    std::shared_ptr<FrEquilibriumFrame_> eqFrame;

protected:
    void SetUp() override;
    void LoadData(std::string filename);

public:

};

void TestLinearHydrostaticForce_::SetUp() {
    body = std::make_shared<FrBody_>();
    system.AddBody(body);
    eqFrame = std::make_shared<FrEquilibriumFrame_>(body.get());
}

void TestLinearHydrostaticForce_::LoadData(std::string filename) {

}