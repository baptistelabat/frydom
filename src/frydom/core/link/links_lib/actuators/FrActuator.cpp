//
// Created by frongere on 06/02/19.
//

#include "FrActuator.h"

#include "frydom/core/link/links_lib/FrLink.h"


namespace frydom {


    FrActuator::FrActuator(FrLink_ *associatedLink) :
        FrLinkBase_(associatedLink->GetNode1(), associatedLink->GetNode2(), associatedLink->GetSystem()),
        m_associatedLink(associatedLink) {}



}  // end namespace frydom
