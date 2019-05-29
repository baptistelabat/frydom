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


#ifndef FRYDOM_FRCURRENTSTANDARDFORCE_H
#define FRYDOM_FRCURRENTSTANDARDFORCE_H

#include "frydom/core/force/FrForce.h"

namespace frydom {


    /// Standard current drag force from DNV standard
    /// DNV-GL Station Keeping 01111

    /**
     * \class FrCurrentStandardForce
     * \brief Class for computing current loads following the DNV standard.
     */
    class FrCurrentStandardForce : public FrForce {

    private:
        double m_breadth = -1;
        double m_draft = -1;
        double m_lateralArea = -1;
        double m_transverseArea =-1 ;
        double m_xCenter;
        double m_lpp = -1;

    public:

        FrCurrentStandardForce() = default;

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "CurrentStandardForce"; }

        void SetMaximumBreadth(double breadth);

        double GetMaximumBreadth() const { return m_breadth; }

        void SetDraft(double draft);

        double GetDraft() const { return m_draft; }

        void SetLateralArea(double lateralArea);

        double GetLateralArea() const { return m_lateralArea; }

        void SetTransverseArea(double transverseArea);

        double GetTransverseArea() const { return m_transverseArea; }

        void SetXCenter(double xCenter);

        double GetXCenter() const { return m_xCenter; }

        void SetLengthBetweenPerpendicular(double lpp);

        double GetLengthBetweenPerpendicular() const { return m_lpp; }

        void Initialize() override;

    private:

        void Compute(double time) override;
    };


}  // end namespace frydom


#endif //FRYDOM_FRCURRENTSTANDARDFORCE_H
