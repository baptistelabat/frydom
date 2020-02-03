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

  class FrLink;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrLink, "LINKS/LINK_");

  class FrConstraint;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrConstraint, "CONSTRAINTS/CONSTRAINT_");

  class FrActuator;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrActuator, "ACTUATORS/ACTUATOR_");

  // NODES

  class FrRootNode;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrRootNode, "ROOT_");

  class FrDynamicCable;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrDynamicCable, "CABLES/CAT_CABLE_");

  class FrCatenaryLine;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrCatenaryLine, "CABLES/DYN_CABLE_");

  //

  class FrEquilibriumFrame;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrEquilibriumFrame, "")

}  // end namespace frydom
