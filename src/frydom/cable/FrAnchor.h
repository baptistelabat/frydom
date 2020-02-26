//
// Created by frongere on 26/02/2020.
//

#ifndef FRYDOM_FRANCHOR_H
#define FRYDOM_FRANCHOR_H

#include "frydom/core/common/FrNode.h"


namespace frydom {

  class FrAnchor : public FrNode {

   public:
    FrAnchor(const std::string &name, FrBody *body) : FrNode(name, body) {}

    FrAnchor(const FrAnchor &other) : FrAnchor(other.GetName(), other.GetBody()) {}

    explicit FrAnchor(const FrNode &other) : FrAnchor(other.GetName(), other.GetBody()) {}

  };

}  // end namespace frydom



#endif //FRYDOM_FRANCHOR_H
