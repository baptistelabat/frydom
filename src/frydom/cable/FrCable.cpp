//
// Created by frongere on 10/10/17.
//

#include "FrCable.h"

namespace frydom {

    FrCable_::FrCable_() = default;

    FrCable_::FrCable_(const std::shared_ptr<FrNode_> startingNode, const std::shared_ptr<FrNode_> endingNode,
                       const double cableLength, const double youngModulus, const double sectionArea,
                       const double linearDensity)
            : m_startNode(startingNode),
              m_endNode(endingNode),
              m_cableLength(cableLength),
              m_unrollingSpeed(0.),
              m_youngModulus(youngModulus),
              m_sectionArea(sectionArea),
              m_linearDensity(linearDensity) {}


    FrCable_::~FrCable_() = default;

    FrOffshoreSystem_* FrCable_::GetSystem() {
        return m_system;
    }

}