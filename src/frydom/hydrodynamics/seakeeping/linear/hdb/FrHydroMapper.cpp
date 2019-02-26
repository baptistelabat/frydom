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


#include "FrHydroMapper.h"
//#include "FrBEMBody.h"
//
//#include "frydom/hydrodynamics/FrEquilibriumFrame.h"


namespace frydom {

    void FrHydroMapper_::Map(FrBEMBody_* BEMBody, FrBody_* body, std::shared_ptr<FrEquilibriumFrame_> eqFrame) {
        m_mapBEMToBody[BEMBody] = body;
        m_mapBodyToBEM[body] = BEMBody;
        m_mapEqFrame[BEMBody] = eqFrame;
    }

    unsigned long FrHydroMapper_::GetNbMappings() const {
        return m_mapBEMToBody.size();
    }

    FrBody_* FrHydroMapper_::GetBody(FrBEMBody_* BEMBody) const {
        return m_mapBEMToBody.at(BEMBody);
    }

    FrBEMBody_* FrHydroMapper_::GetBEMBody(FrBody_* body) const {
        return m_mapBodyToBEM.at(body);
    }

    unsigned int FrHydroMapper_::GetBEMBodyIndex(FrBody_* body) const {
        auto BEMBody = m_mapBodyToBEM.at(body);
        return BEMBody->GetID();
    }

    FrEquilibriumFrame_* FrHydroMapper_::GetEquilibriumFrame(FrBEMBody_* BEMBody) const {
        return m_mapEqFrame.at(BEMBody).get();
    }

    FrEquilibriumFrame_* FrHydroMapper_::GetEquilibriumFrame(FrBody_* body) const {
        auto BEMBody = this->GetBEMBody(body);
        return m_mapEqFrame.at(BEMBody).get();
    }


}  // end namespace frydom
