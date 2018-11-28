//
// Created by frongere on 29/10/18.
//

#include "FrMorisonForce.h"

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

    void FrMorisonForce_::Update(double time) {

        m_model->Update(time);

        SetForceInWorldAtCOG(m_model->GetForceInWorld(NWU), NWU);
        SetTorqueInBodyAtCOG(m_model->GetTorqueInBody(), NWU);
    }

    void FrMorisonForce_::Initialize() {
        m_model->Initialize();
    }

    void FrMorisonForce_::StepFinalize() {

    }


}  // end namespace frydom