//
// Created by frongere on 11/01/18.
//

#include "FrHydroMapper.h"
#include "FrBEMBody.h"


namespace frydom {


    FrHydroMapper::FrHydroMapper(FrHydroDB *HDB) : m_HDB(HDB) {}

    void FrHydroMapper::Map(const std::shared_ptr<FrHydroBody> hydroBody, unsigned int iBEMBody) {
        // TODO: mettre un safe guard pour ne pas attacher plusieurs corps a un meme BEMBody (meme indice)
        assert(iBEMBody < m_HDB->GetNbBodies());
        m_mapper.insert( mapping(hydroBody.get(), iBEMBody) );
        GetBEMBody(hydroBody)->SetHydroBody(hydroBody.get());
    }

    unsigned int FrHydroMapper::GetNbMappings() const {
        return m_mapper.size();
    }

    FrHydroBody *FrHydroMapper::GetHydroBody(unsigned int iBEMBody) const {
        return m_mapper.right.at(iBEMBody);
    }

    unsigned int FrHydroMapper::GetBEMBodyIndex(std::shared_ptr<FrHydroBody> hydroBody) {
        return GetBEMBodyIndex(hydroBody.get());
    }

    unsigned int FrHydroMapper::GetBEMBodyIndex(FrHydroBody *hydroBody) {
        return m_mapper.left.at(hydroBody);
    }

    std::shared_ptr<FrBEMBody> FrHydroMapper::GetBEMBody(std::shared_ptr<FrHydroBody> hydroBody) {
        return m_HDB->GetBody(GetBEMBodyIndex(hydroBody.get()));
    }

    std::shared_ptr<FrBEMBody> FrHydroMapper::GetBEMBody(FrHydroBody *hydroBody) {
        return m_HDB->GetBody(GetBEMBodyIndex(hydroBody));
    }

    void FrHydroMapper::IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<> &R,
                                           const chrono::ChVectorDynamic<> &w, const double c) {
        m_HDB->IntLoadResidual_Mv(off, R, w, c);
    }























    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    void FrHydroMapper_::Map(FrBEMBody_* BEMBody, FrBody_* body) {
        mapBEMToBody[BEMBody] = body;
        mapBodyToBEM[body] = BEMBody;
    }

    void FrHydroMapper_::Map(FrBody_* body, FrBEMBody_* BEMBody) {
        this->Map(BEMBody, body);
    }

    unsigned long FrHydroMapper_::GetNbMappings() const {
        return mapBEMToBody.size();
    }

    FrBody_* FrHydroMapper_::GetBody(FrBEMBody_* BEMBody) const {
        return mapBEMToBody[BEMBody];
    }

    FrBEMBody_* FrHydroMapper_::GetBEMBody(FrBody_* body) const {
        return mapBodyToBEM[body];
    }

    unsigned int FrHydroMapper_::GetBEMBodyIndex(FrBody_* body) const {
        auto BEMBody = mapBodyToBEM[body];
        return BEMBody->GetID();
    }


}  // end namespace frydom