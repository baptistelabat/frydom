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

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"


namespace frydom {


//    FrHydroMapper::FrHydroMapper(FrHydroDB *HDB) : m_HDB(HDB) {}
//
//    void FrHydroMapper::Map(const std::shared_ptr<FrHydroBody> hydroBody, unsigned int iBEMBody) {
//        // TODO: mettre un safe guard pour ne pas attacher plusieurs corps a un meme BEMBody (meme indice)
//        assert(iBEMBody < m_HDB->GetNbBodies());
//        m_mapper.insert( mapping(hydroBody.get(), iBEMBody) );
//        GetBEMBody(hydroBody)->SetHydroBody(hydroBody.get());
//    }
//
//    unsigned int FrHydroMapper::GetNbMappings() const {
//        return m_mapper.size();
//    }
//
//    FrHydroBody *FrHydroMapper::GetHydroBody(unsigned int iBEMBody) const {
//        return m_mapper.right.at(iBEMBody);
//    }
//
//    unsigned int FrHydroMapper::GetBEMBodyIndex(std::shared_ptr<FrHydroBody> hydroBody) {
//        return GetBEMBodyIndex(hydroBody.get());
//    }
//
//    unsigned int FrHydroMapper::GetBEMBodyIndex(FrHydroBody *hydroBody) {
//        return m_mapper.left.at(hydroBody);
//    }
//
//    std::shared_ptr<FrBEMBody> FrHydroMapper::GetBEMBody(std::shared_ptr<FrHydroBody> hydroBody) {
//        return m_HDB->GetBody(GetBEMBodyIndex(hydroBody.get()));
//    }
//
//    std::shared_ptr<FrBEMBody> FrHydroMapper::GetBEMBody(FrHydroBody *hydroBody) {
//        return m_HDB->GetBody(GetBEMBodyIndex(hydroBody));
//    }
//
//    void FrHydroMapper::IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<> &R,
//                                           const chrono::ChVectorDynamic<> &w, const double c) {
//        m_HDB->IntLoadResidual_Mv(off, R, w, c);
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
//
//    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    void FrHydroMapper_::Map(FrBEMBody_* BEMBody, FrBody_* body, std::shared_ptr<FrEquilibriumFrame_> eqFrame) {

        /// This subroutine makes the mapping between a FrBody, a BEMBody and a FrEquilibriumFrame.

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
