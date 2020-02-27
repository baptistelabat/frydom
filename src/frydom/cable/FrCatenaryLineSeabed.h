//
// Created by frongere on 27/02/2020.
//

#ifndef FRYDOM_FRCATENARYLINESEABED_H
#define FRYDOM_FRCATENARYLINESEABED_H

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/asset/FrCatenaryLineAsset.h"
#include "frydom/environment/FrFluidType.h"
#include "FrCatenaryLine.h"


namespace frydom {

  class FrCatenaryLineSeabed : public FrCatenaryLine {

   public:
    FrCatenaryLineSeabed(const std::string &name,
                         const std::shared_ptr<FrNode> &anchorNode,
                         const std::shared_ptr<FrNode> &fairleadNode,
                         const std::shared_ptr<FrCableProperties> &properties,
                         bool elastic,
                         double unstretchedLength,
                         FLUID_TYPE fluid);

    void Initialize() override;

    void solve() override;

    Force GetTensionAtTouchDownPoint() {
      return m_t0;
    }

   private:

    FrNode *GetAnchorNode() {
      return m_anchor_node.get();
    }

    FrNode *GetFairleadNode() {
      return m_endingNode.get();
    }

    FrNode *GetTouchDownPointNode() {
      return m_startingNode.get();
    }

    void UpdateTouchDownPointPosition() {
      m_startingNode->SetPositionInWorld(m_anchor_node->GetPositionInWorld(NWU) + m_Lb * m_lying_direction, NWU);
    }

    double GetSeabedIntersection() const;

    Position GetPositionOnSeabed(double s) const;


   private:

    std::shared_ptr<FrNode> m_anchor_node;

    double m_Lb; // Abscisse lagrangienne du TDP
    bool m_use_seabed_interaction_solver;

    Direction m_lying_direction;

    double c_Cb;

  };

}  // end namespace frydom

#endif //FRYDOM_FRCATENARYLINESEABED_H
