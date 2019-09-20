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

    template<typename OffshoreSystemType>
    void
    FrHydroMapper<OffshoreSystemType>::Map(FrBEMBody<OffshoreSystemType> *BEMBody, FrBody<OffshoreSystemType> *body,
                                           std::shared_ptr<FrEquilibriumFrame<OffshoreSystemType>> eqFrame) {
      m_mapBEMToBody[BEMBody] = body;
      m_mapBodyToBEM[body] = BEMBody;
      m_mapEqFrame[BEMBody] = eqFrame;
    }

    template<typename OffshoreSystemType>
    unsigned long FrHydroMapper<OffshoreSystemType>::GetNbMappings() const {
      return m_mapBEMToBody.size();
    }

    template<typename OffshoreSystemType>
    FrBody<OffshoreSystemType> *
    FrHydroMapper<OffshoreSystemType>::GetBody(FrBEMBody<OffshoreSystemType> *BEMBody) const {
      return m_mapBEMToBody.at(BEMBody);
    }

    template<typename OffshoreSystemType>
    FrBEMBody<OffshoreSystemType> *
    FrHydroMapper<OffshoreSystemType>::GetBEMBody(FrBody<OffshoreSystemType> *body) const {
      return m_mapBodyToBEM.at(body);
    }

    template<typename OffshoreSystemType>
    unsigned int FrHydroMapper<OffshoreSystemType>::GetBEMBodyIndex(FrBody<OffshoreSystemType> *body) const {
      auto BEMBody = m_mapBodyToBEM.at(body);
      return BEMBody->GetID();
    }

    template<typename OffshoreSystemType>
    FrEquilibriumFrame<OffshoreSystemType> *
    FrHydroMapper<OffshoreSystemType>::GetEquilibriumFrame(FrBEMBody<OffshoreSystemType> *BEMBody) const {
      return m_mapEqFrame.at(BEMBody).get();
    }

    template<typename OffshoreSystemType>
    FrEquilibriumFrame<OffshoreSystemType> *
    FrHydroMapper<OffshoreSystemType>::GetEquilibriumFrame(FrBody<OffshoreSystemType> *body) const {
      auto BEMBody = this->GetBEMBody(body);
      return m_mapEqFrame.at(BEMBody).get();
    }

}  // end namespace frydom
