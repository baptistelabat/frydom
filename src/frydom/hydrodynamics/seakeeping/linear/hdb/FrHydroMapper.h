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

#include <unordered_map>
#include <memory>


namespace frydom {

    // Forward declaration
    class FrEquilibriumFrame;
    class FrBEMBody;
    class FrBody;
    class FrEquilibriumFrame;

    /**
     * \class FrHydroMapper
     * \brief Class for mapping the bodies with the HDB.
     */
    class FrHydroMapper {

    private:
        std::unordered_map<FrBEMBody*, FrBody*> m_mapBEMToBody; // Mapping of FrBEMBodies with FrBodies.
        std::unordered_map<FrBody*, FrBEMBody*> m_mapBodyToBEM; // Mapping of FrBodies with FrBEMBodies.
        std::unordered_map<FrBEMBody*, std::shared_ptr<FrEquilibriumFrame>> m_mapEqFrame; // Mapping of BEMBodies with FrEquilibriumFrame.

    public:

        /// Constructor.
        FrHydroMapper() = default;

        /// This subroutine makes the mapping between a FrBody, a BEMBody and a FrEquilibriumFrame.
        void Map(FrBEMBody* BEMBody, FrBody* body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

        unsigned long GetNbMappings() const;

        FrBody* GetBody(FrBEMBody* BEMBody) const;

        FrBEMBody* GetBEMBody(FrBody* body) const;

        unsigned int GetBEMBodyIndex(FrBody* body) const;

        FrEquilibriumFrame* GetEquilibriumFrame(FrBEMBody* BEMBody) const;

        FrEquilibriumFrame* GetEquilibriumFrame(FrBody* body) const;

        std::unordered_map<FrBEMBody*, FrBody*>::iterator begin() { return m_mapBEMToBody.begin(); }

        std::unordered_map<FrBEMBody*, FrBody*>::iterator end() { return m_mapBEMToBody.end(); };

    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROMAPPER_H
