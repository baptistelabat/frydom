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

    FrRadiationForce::FrRadiationForce(FrRadiationModel* radiationModel)
            : m_radiationModel(radiationModel) {}

    void FrRadiationForce::SetRadiationModel(FrRadiationModel* radiationModel) {
        m_radiationModel = radiationModel;
    }

    void FrRadiationForce::StepFinalize() {
        FrForce::StepFinalize();
    }

    // --------------------------------------------------
    // FrRadiationConvolutionForce
    // --------------------------------------------------

    FrRadiationConvolutionForce::FrRadiationConvolutionForce(
            FrRadiationConvolutionModel* radiationModel)
            : FrRadiationForce(radiationModel) {}

    void FrRadiationConvolutionForce::Initialize() {
        FrRadiationForce::Initialize();
    }

    void FrRadiationConvolutionForce::Compute(double time) {

        auto force = m_radiationModel->GetRadiationForce(m_body);
        auto torque = m_radiationModel->GetRadiationTorque(m_body);

        SetForceTorqueInWorldAtCOG(force, torque, NWU);
    }

}  // end namespace frydom
