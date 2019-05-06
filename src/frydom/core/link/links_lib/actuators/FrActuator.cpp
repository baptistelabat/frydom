//
// Created by frongere on 06/02/19.
//

#include "FrActuator.h"

#include "frydom/core/link/links_lib/FrLink.h"


namespace frydom {


    FrActuator::FrActuator(FrLink *actuatedLink) :
        FrLinkBase(actuatedLink->GetNode1(), actuatedLink->GetNode2(), actuatedLink->GetSystem()),
        m_actuatedLink(actuatedLink) {}

    bool FrActuator::IsDisabled() const {
        return GetChronoActuator()->GetDisabled(); // TODO : voir si on teste aussi m_actuatedLink
    }

    void FrActuator::SetDisabled(bool disabled) {
        GetChronoActuator()->MakeDisabled(disabled);
    }

//    bool FrActuator::IsBroken() const {
//        return m_chronoMotor->IsBroken();
//    }
//
//    void FrActuator::SetBroken(bool broken) {
//
//    }

    bool FrActuator::IsActive() const {
        return false;
    }





}  // end namespace frydom
