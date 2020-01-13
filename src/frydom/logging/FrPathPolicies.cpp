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

//<<<<<<< HEAD
//  class FrLinkBase;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrLinkBase, "LINKS/LINK_"); // TODO: voir aussi avec les actuateurs...

  class FrLink;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrLink, "LINKS/LINK_");

  class FrConstraint;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrConstraint, "CONSTRAINTS/CONSTRAINT_");

  class FrActuator;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrActuator, "ACTUATORS/ACTUATOR_");
//=======
//  // LINKS
//
//  class FrLinkBase;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrLinkBase, "LINKS/LINK_"); // TODO: voir aussi avec les actuateurs...
//>>>>>>> develop

//  class FrRevoluteLink;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrRevoluteLink, "LINKS/LINK_");
//
//  class FrCylindricalLink;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrCylindricalLink, "LINKS/LINK_");
//
//  class FrDOFMaskLink;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrDOFMaskLink, "LINKS/LINK_");
//
//  class FrFixedLink;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrFixedLink, "LINKS/LINK_");
//
//  class FrFreeLink;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrFreeLink, "LINKS/LINK_");
//
////  class FrLink;
////  TYPE_TO_NORMALIZED_PATH_PREFIX(FrLink, "LINKS/LINK_");
//
//  class FrPrismaticLink;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrPrismaticLink, "LINKS/LINK_");
//
//  class FrPrismaticRevoluteLink;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrPrismaticRevoluteLink, "LINKS/LINK_");
//
//  class FrSphericalLink;
//  TYPE_TO_NORMALIZED_PATH_PREFIX(FrSphericalLink, "LINKS/LINK_");

  // NODES

  class FrRootNode;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrRootNode, "ROOT_");

  class FrDynamicCable;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrDynamicCable, "CABLES/CAT_CABLE_");

  class FrCatenaryLine;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrCatenaryLine, "CABLES/DYN_CABLE_");

}  // end namespace frydom
