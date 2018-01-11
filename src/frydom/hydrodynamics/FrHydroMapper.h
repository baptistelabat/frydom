//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRHYDROMAPPER_H
#define FRYDOM_FRHYDROMAPPER_H

#include "boost/bimap.hpp"

#include "frydom/core/FrHydroBody.h"
#include "FrHydroDB.h"


namespace frydom {

    typedef boost::bimaps::bimap<std::shared_ptr<FrHydroBody>, std::shared_ptr<FrBEMBody>> myBimap;
    typedef myBimap::value_type mapping;


    class FrHydroMapper {

    private:

        FrHydroDB* m_HDB;

        myBimap m_mapper;


    public:

        explicit FrHydroMapper(FrHydroDB* HDB) : m_HDB(HDB) {}

        void Map(std::shared_ptr<FrHydroBody> hydroBody, unsigned int iBEMBody) {

            assert(iBEMBody < m_HDB->GetNbBodies());
            m_mapper.insert( mapping(hydroBody, m_HDB->GetBody(iBEMBody)) );

        }

        unsigned int GetNbMappings() const {
            return m_mapper.size();
        }

        std::shared_ptr<FrHydroBody> GetHydroBody(std::shared_ptr<FrBEMBody> BEMBody) const {
            return m_mapper.right.at(BEMBody);
        }

        std::shared_ptr<FrHydroBody> GetHydroBody(unsigned int iBEMBody) const {
            return m_mapper.right.at(m_HDB->GetBody(iBEMBody));
        }

        std::shared_ptr<FrBEMBody> GetBEMBody(std::shared_ptr<FrHydroBody> hydroBody) {
            return m_mapper.left.at(hydroBody);
        }

    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROMAPPER_H
