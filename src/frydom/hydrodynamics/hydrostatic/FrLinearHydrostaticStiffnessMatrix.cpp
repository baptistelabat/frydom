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


#include "FrLinearHydrostaticStiffnessMatrix.h"

namespace frydom {

    FrLinearHydrostaticStiffnessMatrix::FrLinearHydrostaticStiffnessMatrix() : m_data(mathutils::Matrix33<double>()) {}

    void FrLinearHydrostaticStiffnessMatrix::SetK33(double K33) { m_data.at(0, 0) = K33; }

    void FrLinearHydrostaticStiffnessMatrix::SetK44(double K44) { m_data.at(1, 1) = K44; }

    void FrLinearHydrostaticStiffnessMatrix::SetK55(double K55) { m_data.at(2, 2) = K55; }

    void FrLinearHydrostaticStiffnessMatrix::SetK34(double K34) { m_data.at(0, 1) = K34; m_data.at(1, 0) = K34; }

    void FrLinearHydrostaticStiffnessMatrix::SetK35(double K35) { m_data.at(0, 2) = K35; m_data.at(2, 0) = K35; }

    void FrLinearHydrostaticStiffnessMatrix::SetK45(double K45) { m_data.at(1, 2) = K45; m_data.at(2, 1) = K45; }

    double FrLinearHydrostaticStiffnessMatrix::GetK33() const { return m_data.at(0, 0); }

    double FrLinearHydrostaticStiffnessMatrix::GetK44() const { return m_data.at(1, 1); }

    double FrLinearHydrostaticStiffnessMatrix::GetK55() const { return m_data.at(2, 2); }

    double FrLinearHydrostaticStiffnessMatrix::GetK34() const { return m_data.at(0, 1); }

    double FrLinearHydrostaticStiffnessMatrix::GetK35() const { return m_data.at(0, 2); }

    double FrLinearHydrostaticStiffnessMatrix::GetK45() const { return m_data.at(1, 2); }

    mathutils::Matrix33<double> FrLinearHydrostaticStiffnessMatrix::GetMatrix(){

        // This function gives the hydrostatic stiffness matrix.

        mathutils::Matrix33<double> mat;
        mat(0, 0) = this->GetK33();
        mat(1, 1) = this->GetK44();
        mat(2, 2) = this->GetK55();
        mat(0, 1) = mat(1, 0) = this->GetK34();
        mat(0, 2) = mat(2, 0) = this->GetK35();
        mat(1, 2) = mat(2, 1) = this->GetK45();

        return mat;

    }

    void FrLinearHydrostaticStiffnessMatrix::SetDiagonal(double K33, double K44, double K55) {
        this->SetK33(K33);
        this->SetK44(K44);
        this->SetK55(K55);
    }

    void FrLinearHydrostaticStiffnessMatrix::SetNonDiagonal(double K34, double K35, double K45) {
        this->SetK34(K34);
        this->SetK35(K35);
        this->SetK45(K45);
    }

    void
    FrLinearHydrostaticStiffnessMatrix::SplitCoeffs(double &K33, double &K44, double &K55, double &K34, double &K35,
                                                     double &K45) {
        K33 = this->GetK33();
        K44 = this->GetK44();
        K55 = this->GetK55();
        K34 = this->GetK34();
        K35 = this->GetK35();
        K45 = this->GetK45();
    }

    // This subroutine sets the reduced hydrostatic matrix (3x3).
    void FrLinearHydrostaticStiffnessMatrix::SetData(const mathutils::Matrix33<double> &data) { m_data = data; }

    mathutils::Vector3d<double>
    FrLinearHydrostaticStiffnessMatrix::operator*(const mathutils::Vector3d<double> &state) const {
        return m_data * state;
    }

}  // end namespace frydom
