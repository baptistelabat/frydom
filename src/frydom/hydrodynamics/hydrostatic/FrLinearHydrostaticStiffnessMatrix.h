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

#include "MathUtils/Matrix33.h"
#include "MathUtils/Vector3d.h"


namespace frydom {

    /**
     * \class FrLinearHydrostaticStiffnessMatrix
     * \brief Class for defning the hydrostatic stiffness matrix.
     */
    class FrLinearHydrostaticStiffnessMatrix {

    protected:
        mathutils::Matrix33<double> m_data;

    public:
        FrLinearHydrostaticStiffnessMatrix();

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

        /// This function gives the hydrostatic stiffness matrix.
        mathutils::Matrix33<double> GetMatrix();

        void SetDiagonal(double K33, double K44, double K55);

        void SetNonDiagonal(double K34, double K35, double K45);

        void SplitCoeffs(double& K33, double& K44, double& K55,
                         double& K34, double& K35, double& K45);

        // This subroutine sets the reduced hydrostatic matrix (3x3).
        void SetData(const mathutils::Matrix33<double>& data);

        mathutils::Vector3d<double> operator*(const mathutils::Vector3d<double>& state) const;

    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEARHYDROSTATICSTIFFNESSMATRIX_H
