//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRHYDROMAPPER_H
#define FRYDOM_FRHYDROMAPPER_H

#include "boost/bimap.hpp"

#include "frydom/core/junk/FrHydroBody.h"
#include "FrHydroDB.h"
#include "frydom/hydrodynamics/FrBEMBody.h"


namespace frydom {

    typedef boost::bimaps::bimap<FrHydroBody*, unsigned int> myBimap;
    typedef myBimap::value_type mapping;


    class FrHydroMapper {

    private:

        FrHydroDB* m_HDB;
        myBimap m_mapper;


    public:

        explicit FrHydroMapper(FrHydroDB* HDB);

        void Map(const std::shared_ptr<FrHydroBody> hydroBody, unsigned int iBEMBody);

        unsigned int GetNbMappings() const;

        FrHydroBody* GetHydroBody(unsigned int iBEMBody) const;

        unsigned int GetBEMBodyIndex(std::shared_ptr<FrHydroBody> hydroBody);

        unsigned int GetBEMBodyIndex(FrHydroBody* hydroBody);

        std::shared_ptr<FrBEMBody> GetBEMBody(std::shared_ptr<FrHydroBody> hydroBody);

        std::shared_ptr<FrBEMBody> GetBEMBody(FrHydroBody* hydroBody);

        virtual void IntLoadResidual_Mv(const unsigned int off,
                                        chrono::ChVectorDynamic<>& R,
                                        const chrono::ChVectorDynamic<>& w,
                                        const double c);

        //virtual void VariablesFbIncrementMq() { m_HDB->VariablesFbIncrementMq(); }


    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROMAPPER_H
