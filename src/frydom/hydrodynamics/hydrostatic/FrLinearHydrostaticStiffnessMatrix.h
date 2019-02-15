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


#ifndef FRYDOM_FRLINEARHYDROSTATICSTIFFNESSMATRIX_H
#define FRYDOM_FRLINEARHYDROSTATICSTIFFNESSMATRIX_H

// TODO: mettre en place les methodes de transport des raideurs dans d'autres reperes...

#include <chrono/core/ChVector.h>

#include "frydom/core/math/FrMatrix.h"
#include "MathUtils/Vector3d.h"

namespace frydom {

    /**
     * \class FrLinearHydrostaticStiffnessMatrix
     * \brief Class for defning the hydrostatic restoring matrix.
     */
    class FrLinearHydrostaticStiffnessMatrix {

    private:
        chrono::ChVector<double> m_stiffnessDiagonal;
        chrono::ChVector<double> m_stiffnessNonDiagonal;

    public:
        FrLinearHydrostaticStiffnessMatrix() :
                m_stiffnessDiagonal(chrono::VNULL),
                m_stiffnessNonDiagonal(chrono::VNULL) {};

        FrLinearHydrostaticStiffnessMatrix(const chrono::ChVector<double>& KhXX,
                                           const chrono::ChVector<double>& KhYY)
                : m_stiffnessDiagonal(KhXX), m_stiffnessNonDiagonal(KhYY) {}

        void SetK33(double K33) {m_stiffnessDiagonal.x() = K33; }

        const double& GetK33() const { return m_stiffnessDiagonal.x(); }

        void SetK44(double K44) {m_stiffnessDiagonal.y() = K44; }

        const double& GetK44() const { return m_stiffnessDiagonal.y(); }

        void SetK55(double K55) {m_stiffnessDiagonal.z() = K55; }

        const double& GetK55() const { return m_stiffnessDiagonal.z(); }

        void SetK34(double K34) {m_stiffnessDiagonal.x() = K34; }

        const double& GetK34() const { return m_stiffnessNonDiagonal.x(); }

        void SetK35(double K35) {m_stiffnessDiagonal.y() = K35; }

        const double& GetK35() const { return m_stiffnessNonDiagonal.y(); }

        void SetK45(double K45) {m_stiffnessDiagonal.z() = K45; }

        const double& GetK45() const { return m_stiffnessNonDiagonal.z(); }

        void SetDiagonal(double K33, double K44, double K55) {
            m_stiffnessDiagonal = chrono::ChVector<double>(K33, K44, K55);
        }

        void SetDiagonal(const chrono::ChVector<double> &stiffness) { m_stiffnessDiagonal = stiffness; }

        void SetNonDiagonal(double K34, double K35, double K45) {
            m_stiffnessNonDiagonal = chrono::ChVector<double>(K34, K35, K45);
        }

        void SetNonDiagonal(const chrono::ChVector<double>& stiffness) { m_stiffnessNonDiagonal = stiffness; }



        void SplitCoeffs(double& K33, double& K44, double& K55,
                         double& K34, double& K35, double& K45) {

            K33 = m_stiffnessDiagonal.x();
            K44 = m_stiffnessDiagonal.y();
            K55 = m_stiffnessDiagonal.z();
            K34 = m_stiffnessNonDiagonal.x();
            K35 = m_stiffnessNonDiagonal.y();
            K45 = m_stiffnessNonDiagonal.z();

        }

        chrono::ChVector<double> operator*(const chrono::ChVector<double>& posState) {

            double K33, K44, K55, K34, K35, K45;
            SplitCoeffs(K33, K44, K55, K34, K35, K45);

            double dHeave, dRoll, dPitch;
            dHeave = posState.x();
            dRoll = posState.y();
            dPitch = posState.z();

            return chrono::ChVector<double>(
                    K33 * dHeave + K34 * dRoll + K35 * dPitch,
                    K34 * dHeave + K44 * dRoll + K45 * dPitch,
                    K35 * dHeave + K45 * dRoll + K55 * dPitch
            );

        }



    };

























    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    /// This class defines the hydrostatic stiffness matrix for the linear hydrostatic force
    /// Only non null values are stored in the data.

    /**
     * \class FrLinearHydrostaticStiffnessMatrix_
     * \brief Class for defning the hydrostatic restoring matrix.
     */
    class FrLinearHydrostaticStiffnessMatrix_ {

    protected:
        mathutils::Matrix33<double> m_data;

    public:
        FrLinearHydrostaticStiffnessMatrix_();

        void SetK33(double K33);
        void SetK44(double K44);
        void SetK55(double K55);
        void SetK34(double K34);
        void SetK35(double K35);
        void SetK45(double K45);

        double GetK33() const;
        double GetK44() const;
        double GetK55() const;
        double GetK34() const;
        double GetK35() const;
        double GetK45() const;

        void SetDiagonal(double K33, double K44, double K55);

        void SetNonDiagonal(double K34, double K35, double K45);

        void SplitCoeffs(double& K33, double& K44, double& K55,
                         double& K34, double& K35, double& K45);

        void SetData(const mathutils::Matrix33<double>& data);

        mathutils::Vector3d<double> operator*(const mathutils::Vector3d<double>& state) const;

    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEARHYDROSTATICSTIFFNESSMATRIX_H
