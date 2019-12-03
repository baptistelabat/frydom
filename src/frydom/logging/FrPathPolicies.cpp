//
// Created by frongere on 03/12/2019.
//

#include "FrPathPolicies.h"


namespace frydom {

  class FrOffshoreSystem;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrOffshoreSystem, "FRYDOM_");

  class FrBody;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrBody, "BODIES/BODY_");

  class FrForce;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrForce, "FORCES/FORCE_");

  class FrNode;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrNode, "NODES/NODE_");

  class FrLinkBase;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrLinkBase, "LINKS/LINK_"); // TODO: voir aussi avec les actuateurs...

  class FrRootNode;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrRootNode, "ROOT_");

  class FrDynamicCable;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrDynamicCable, "CABLES/DYN_CABLE_");

}  // end namespace frydom
