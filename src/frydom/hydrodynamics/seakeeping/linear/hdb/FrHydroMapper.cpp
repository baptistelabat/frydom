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
#include "FrBEMBody.h"


namespace frydom {

    void FrHydroMapper::Map(FrBEMBody* BEMBody, FrBody* body, std::shared_ptr<FrEquilibriumFrame> eqFrame) {
        m_mapBEMToBody[BEMBody] = body;
        m_mapBodyToBEM[body] = BEMBody;
        m_mapEqFrame[BEMBody] = eqFrame;
    }

    unsigned long FrHydroMapper::GetNbMappings() const {
        return m_mapBEMToBody.size();
    }

    FrBody* FrHydroMapper::GetBody(FrBEMBody* BEMBody) const {
        return m_mapBEMToBody.at(BEMBody);
    }

    FrBEMBody* FrHydroMapper::GetBEMBody(FrBody* body) const {
        return m_mapBodyToBEM.at(body);
    }

    unsigned int FrHydroMapper::GetBEMBodyIndex(FrBody* body) const {
        auto BEMBody = m_mapBodyToBEM.at(body);
        return BEMBody->GetID();
    }

    FrEquilibriumFrame* FrHydroMapper::GetEquilibriumFrame(FrBEMBody* BEMBody) const {
        return m_mapEqFrame.at(BEMBody).get();
    }

    FrEquilibriumFrame* FrHydroMapper::GetEquilibriumFrame(FrBody* body) const {
        auto BEMBody = this->GetBEMBody(body);
        return m_mapEqFrame.at(BEMBody).get();
    }

}  // end namespace frydom
