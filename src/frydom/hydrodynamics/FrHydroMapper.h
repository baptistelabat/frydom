//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRHYDROMAPPER_H
#define FRYDOM_FRHYDROMAPPER_H

#include "boost/bimap.hpp"

#include "frydom/core/FrHydroBody.h"
#include "FrHydroDB.h"


namespace frydom {

    typedef boost::bimaps::bimap<FrHydroBody*, unsigned int> myBimap;
    typedef myBimap::value_type mapping;


    class FrHydroMapper {

    private:

        FrHydroDB* m_HDB;
        myBimap m_mapper;


    public:

        explicit FrHydroMapper(FrHydroDB* HDB) : m_HDB(HDB) {}

        void Map(const std::shared_ptr<FrHydroBody> hydroBody, unsigned int iBEMBody) {
            // TODO: mettre un safe guard pour ne pas attacher plusieurs corps a un meme BEMBody (meme indice)
            assert(iBEMBody < m_HDB->GetNbBodies());
            m_mapper.insert( mapping(hydroBody.get(), iBEMBody) );

        }

        unsigned int GetNbMappings() const {
            return m_mapper.size();
        }

        FrHydroBody* GetHydroBody(unsigned int iBEMBody) const {
            return m_mapper.right.at(iBEMBody);
        }

        unsigned int GetBEMBodyIndex(std::shared_ptr<FrHydroBody> hydroBody) {
            return GetBEMBodyIndex(hydroBody.get());
        }

        unsigned int GetBEMBodyIndex(FrHydroBody* hydroBody) {
            return m_mapper.left.at(hydroBody);
        }

        std::shared_ptr<FrBEMBody> GetBEMBody(std::shared_ptr<FrHydroBody> hydroBody) {
            return m_HDB->GetBody(GetBEMBodyIndex(hydroBody.get()));
        }

        std::shared_ptr<FrBEMBody> GetBEMBody(FrHydroBody* hydroBody) {
            return m_HDB->GetBody(GetBEMBodyIndex(hydroBody));
        }

    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROMAPPER_H
