//
// Created by frongere on 30/01/2020.
//

#ifndef FRYDOM_FRLUMPEDMASSCABLE_H
#define FRYDOM_FRLUMPEDMASSCABLE_H

#include "FrCable.h"
#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {

  // Forward declaration
  class FrNode;

  class FrLumpedMassCable : public FrLoggable<FrOffshoreSystem>, public FrCable {

    FrLumpedMassCable(const std::string &name,
                      FrOffshoreSystem *system,
                      const std::shared_ptr<FrNode> &startingNode,
                      const std::shared_ptr<FrNode> &endingNode,
                      const std::shared_ptr<FrCableProperties> &properties,
                      double unstretchedLength,
                      double rayleightDamping,
                      unsigned int nbElements
    );

    Force GetTension(double s, FRAME_CONVENTION fc) const override;

    Position GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const override;

    void DefineLogMessages() override;


  };

}  // end namespace frydom



#endif //FRYDOM_FRLUMPEDMASSCABLE_H
