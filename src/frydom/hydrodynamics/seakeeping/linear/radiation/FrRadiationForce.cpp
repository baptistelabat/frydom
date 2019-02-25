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


//    FrRadiationForce::FrRadiationForce(std::shared_ptr<FrRadiationModel> radiationModel) : m_radiationModel(radiationModel) {}
//
//    void FrRadiationForce::SetRadiationModel(const std::shared_ptr<FrRadiationModel> radiationModel) { m_radiationModel = radiationModel; }
//
//    std::shared_ptr<FrRadiationModel> FrRadiationForce::GetRadiationModel() const { return m_radiationModel; }
//
//    void FrRadiationForce::SetLogPrefix(std::string prefix_name) {
//        if (prefix_name=="") {
//            m_logPrefix = "Frad_" + FrForce::m_logPrefix;
//        } else {
//            m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
//        }
//    }
//
//    FrRadiationConvolutionForce::FrRadiationConvolutionForce(
//            std::shared_ptr<FrRadiationConvolutionModel> radiationConvolutionModel)
//            : FrRadiationForce(radiationConvolutionModel) {}
//
//    void FrRadiationConvolutionForce::Initialize() {
//        m_radiationModel->Initialize();
//        FrRadiationForce::Initialize();
//    }
//
//    void FrRadiationConvolutionForce::UpdateState() {
//        // TODO: appeler le Update du RadiationModel
//        m_radiationModel->Update(ChTime);  // TODO: verifier que le ChTime est le bon temps courant !!
//
//        // Current Hydrodynamic body
//        auto hydroBody = dynamic_cast<FrHydroBody*>(GetBody());
//
//        // Get the forces
//        m_radiationModel->GetRadiationForce(hydroBody, force, moment);
//
//        moment = hydroBody->Dir_World2Body(moment);  // Moment expressed in the local coordinate frame
//        // TODO: verifier que c'est la bonne fonction
//
//        // moment in local
////            force = m_radiationModel->GetRadiationForce(hydroBody);
////            moment = m_radiationModel->GetRadiationMoment(hydroBody);
//
//    }
//
//
//
//
//
//
//
//
//
//
//
//    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

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
