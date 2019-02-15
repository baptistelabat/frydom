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


#ifndef FRYDOM_FRHYDROMAPPER_H
#define FRYDOM_FRHYDROMAPPER_H

#include "boost/bimap.hpp"

#include "frydom/core/junk/FrHydroBody.h"
#include "FrHydroDB.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrBEMBody.h"



namespace frydom {

    typedef boost::bimaps::bimap<FrHydroBody*, unsigned int> myBimap;
    typedef myBimap::value_type mapping;

    /**
     * \class FrHydroMapper
     * \brief Class for mapping the bodies with the HDB.
     */
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















    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<< REFACTORING

    class FrEquilibriumFrame_;

    /**
     * \class FrHydroMapper_
     * \brief Class for mapping the bodies with the HDB.
     */
    class FrHydroMapper_ {

    private:
        std::unordered_map<FrBEMBody_*, FrBody_*> m_mapBEMToBody;
        std::unordered_map<FrBody_*, FrBEMBody_*> m_mapBodyToBEM;
        std::unordered_map<FrBEMBody_*, std::shared_ptr<FrEquilibriumFrame_>> m_mapEqFrame;

    public:

        FrHydroMapper_() = default;

        void Map(FrBEMBody_* BEMBody, FrBody_* body, std::shared_ptr<FrEquilibriumFrame_> eqFrame);

        unsigned long GetNbMappings() const;

        FrBody_* GetBody(FrBEMBody_* BEMBody) const;

        FrBEMBody_* GetBEMBody(FrBody_* body) const;

        unsigned int GetBEMBodyIndex(FrBody_* body) const;

        FrEquilibriumFrame_* GetEquilibriumFrame(FrBEMBody_* BEMBody) const;

        FrEquilibriumFrame_* GetEquilibriumFrame(FrBody_* body) const;

    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROMAPPER_H
