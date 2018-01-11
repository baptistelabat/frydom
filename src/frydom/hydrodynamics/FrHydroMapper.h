//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRHYDROMAPPER_H
#define FRYDOM_FRHYDROMAPPER_H

#include "boost/bimap.hpp"

#include "frydom/core/FrHydroBody.h"
#include "FrHydroDB.h"


namespace frydom {

    typedef boost::bimaps::bimap<FrHydroBody*, FrBEMBody*> myBimap;
    typedef myBimap::value_type mapping;


    class FrHydroMapper {

    private:

        FrHydroDB* m_HDB;

        myBimap m_mapper;


    public:

        explicit FrHydroMapper(FrHydroDB* HDB) : m_HDB(HDB) {}

        void Map(std::shared_ptr<FrHydroBody> hydroBody, unsigned int iBEMBody) {

            assert(iBEMBody < m_HDB->GetNbBodies());
            m_mapper.insert( mapping(hydroBody.get(), m_HDB->GetBody(iBEMBody).get()) );

        }

        unsigned int GetNbMappings() const {
            return m_mapper.size();
        }

        FrHydroBody* GetHydroBody(std::shared_ptr<FrBEMBody> BEMBody) const {
            return m_mapper.right.at(BEMBody.get());
        }

        FrHydroBody* GetHydroBody(FrBEMBody* BEMBody) const {
            return m_mapper.right.at(BEMBody);
        }

        FrHydroBody* GetHydroBody(unsigned int iBEMBody) const {
            return m_mapper.right.at(m_HDB->GetBody(iBEMBody).get());
        }

        FrBEMBody* GetBEMBody(std::shared_ptr<FrHydroBody> hydroBody) {
            return m_mapper.left.at(hydroBody.get());
        }
        FrBEMBody* GetBEMBody(FrHydroBody* hydroBody) {
            return m_mapper.left.at(hydroBody);
        }

    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROMAPPER_H
