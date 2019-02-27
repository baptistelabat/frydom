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


#ifndef FRYDOM_FRLINEARDAMPING_H
#define FRYDOM_FRLINEARDAMPING_H

#include "MathUtils/Matrix66.h"

#include "frydom/core/force/FrForce.h"
#include "frydom/environment/FrFluidType.h"



namespace frydom {


    /**
     * \class FrLinearDamping_
     * \brief Class for computing additional linear damping loads.
     */
    class FrLinearDamping : public FrForce {

    public:
        using DampingMatrix = mathutils::Matrix66<double>; // TODO : disposer d'une Matrix66 dans mathutils

    private:

        DampingMatrix m_dampingMatrix;
        FLUID_TYPE m_fluidType;
        bool m_relativeToFluid = false;  // FIXME : on doit pouvoir aussi appliquer dans l'air !!!!!

    public:

        FrLinearDamping(FLUID_TYPE ft, bool relativeToFluid);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "LinearDamping"; }

        void SetNull();

        /// Setter for the whole damping matrix. Translations are upper left and rotations are lower right.
        void SetDampingMatrix(const DampingMatrix& dampingMatrix);

        /// Setter for the diagonal components of the damping matrix
        void SetDiagonalDamping(double Du, double Dv, double Dw, double Dp, double Dq, double Dr);

        /// Setter for the diagonal components in translation of the damping matrix
        void SetDiagonalTranslationDamping(double Du, double Dv, double Dw);

        /// Setter for the diagonal components in rotation of the damping matrix
        void SetDiagonalRotationDamping(double Dp, double Dq, double Dr);

        /// Set a damping coefficient given its position.
        void SetDampingCoeff(unsigned int iRow, unsigned int iCol, double coeff);

        /// Setter for the boolean : m_relativeVelocity
        void SetRelativeToFluid(bool isRelative);

        /// Getter for the boolean : m_relativeVelocity
        bool GetRelativeToFluid();

        void Update(double time) override;

        void Initialize() override;

        void StepFinalize() override;

    private:

        void Check() const;

    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEARDAMPING_H
