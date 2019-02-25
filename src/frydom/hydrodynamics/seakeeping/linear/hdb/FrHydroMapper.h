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

    // Forward declaration
    class FrEquilibriumFrame_;

    /**
     * \class FrHydroMapper_
     * \brief Class for mapping the bodies with the HDB.
     */
    class FrHydroMapper_ {

    private:
        std::unordered_map<FrBEMBody_*, FrBody_*> m_mapBEMToBody; // Mapping of FrBEMBodies with FrBodies.
        std::unordered_map<FrBody_*, FrBEMBody_*> m_mapBodyToBEM; // Mapping of FrBodies with FrBEMBodies.
        std::unordered_map<FrBEMBody_*, std::shared_ptr<FrEquilibriumFrame_>> m_mapEqFrame; // Mapping of BEMBodies with FrEquilibriumFrame.

    public:

        /// Constructor.
        FrHydroMapper_() = default;

        /// This subroutine makes the mapping between a FrBody, a BEMBody and a FrEquilibriumFrame.
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
