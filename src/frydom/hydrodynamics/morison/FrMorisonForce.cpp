// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#include "FrMorisonForce.h"
#include "frydom/hydrodynamics/morison/FrMorisonModel.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {


    frydom::FrMorisonForce::FrMorisonForce() : m_element(NULL) {}

    FrMorisonForce::FrMorisonForce(FrMorisonModel *element) {
        m_element = element;
    }

    void FrMorisonForce::SetElement(FrMorisonModel *element) {
        m_element = element;
    }

    void FrMorisonForce::UpdateState() {
        m_element->UpdateState();
    }

    void FrMorisonForce::UpdateTime(const double time) {
        ChTime = time;
    }

    void FrMorisonForce::Update(const double time) {
        UpdateTime(time);
        UpdateState();
    }

    void FrMorisonForce::Initialize() {

        if(is_log && m_element->LogIsActive()) {
            is_log = true;
        } else {
            is_log = false;
        }

        m_element->Initialize();
        FrForce::Initialize();
    }

    void FrMorisonForce::SetLogPrefix(std::string prefix_name) {
        if (prefix_name=="") {
            m_logPrefix = "Fmorison_" + FrForce::m_logPrefix;
        } else {
            m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
        }
    }

    void FrMorisonForce::SetBodyForce(chrono::ChVector<> mforce) { force = mforce; }

    chrono::ChVector<double> FrMorisonForce::GetBodyForce() const { return force; }

    void FrMorisonForce::SetBodyTorque(chrono::ChVector<> torque) { moment = torque; }

    chrono::ChVector<double> FrMorisonForce::GetBodyTorque() const { return moment; }








    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

    FrMorisonSingleElement_* FrMorisonForce_::SetSingleElementModel(FrBody_* body) {
        m_model = std::make_shared<FrMorisonSingleElement_>(body);
        return dynamic_cast<FrMorisonSingleElement_*>(m_model.get());
    }

    FrMorisonCompositeElement_* FrMorisonForce_::SetCompositeElementModel(FrBody_* body) {
        m_model = std::make_shared<FrMorisonCompositeElement_>(body);
        return dynamic_cast<FrMorisonCompositeElement_*>(m_model.get());
    }

    void FrMorisonForce_::Update(double time) {

        m_model->Update(time);

        SetForceInWorldAtCOG(m_model->GetForceInWorld(NWU), NWU);
        SetTorqueInBodyAtCOG(m_model->GetTorqueInBody(), NWU);
    }

    void FrMorisonForce_::Initialize() {

        FrForce_::Initialize();
        m_model->Initialize();
    }

    void FrMorisonForce_::StepFinalize() {

    }


    std::shared_ptr<FrMorisonForce_>
    make_morison_force(std::shared_ptr<FrMorisonElement_> model, std::shared_ptr<FrBody_> body){
        assert(body.get() == model->GetNode()->GetBody());
        auto MorisonForce = std::make_shared<FrMorisonForce_>(model);
        body->AddExternalForce(MorisonForce);
        return MorisonForce;
    }

}  // end namespace frydom