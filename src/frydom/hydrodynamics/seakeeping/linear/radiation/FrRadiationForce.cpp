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


#include "FrRadiationForce.h"

#include "FrRadiationModel.h"


namespace frydom {

    // --------------------------------------------------
    // FrRadiationForce
    // --------------------------------------------------

    FrRadiationForce_::FrRadiationForce_(FrRadiationModel_* radiationModel)
            : m_radiationModel(radiationModel) {}

    void FrRadiationForce_::SetRadiationModel(FrRadiationModel_* radiationModel) {
        m_radiationModel = radiationModel;
    }

    void FrRadiationForce_::StepFinalize() {
        FrForce_::StepFinalize();
    }

    // --------------------------------------------------
    // FrRadiationConvolutionForce
    // --------------------------------------------------

    FrRadiationConvolutionForce_::FrRadiationConvolutionForce_(
            FrRadiationConvolutionModel_* radiationModel)
            : FrRadiationForce_(radiationModel) {}

    void FrRadiationConvolutionForce_::Initialize() {
        FrRadiationForce_::Initialize();
    }

    void FrRadiationConvolutionForce_::Update(double time) {

        auto force = m_radiationModel->GetRadiationForce(m_body);
        auto torque = m_radiationModel->GetRadiationTorque(m_body);

        SetForceTorqueInWorldAtCOG(force, torque, NWU);
    }

}  // end namespace frydom
