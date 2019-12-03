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

#ifndef FRYDOM_FRPATHMANAGER_H
#define FRYDOM_FRPATHMANAGER_H

#include <unordered_set>
#include <typeinfo>
#include <iostream>

#include "FrPathPolicies.h"


namespace frydom {

  template<class ParentType>
  class FrTreeNode;

  class FrOffshoreSystem;

  class FrBody;

  class FrLinkBase;

  class FrForce;

  class FrNode;

  class FrActuator;


  // TODO : renommer en TreeManager ??
  class FrPathManager {

   public:

    template<class ParentType>
    static std::string GetPath(FrTreeNode<ParentType> *node) {
      if (!node) return "";

      return GetPath(node->GetParent()) + GetNormalizedPathName(node);
    }

    template<class ParentType>
    static std::string GetPath(const FrTreeNode<ParentType> &node) {
      return GetPath(&node);
    }


    template<class ParentType>
    bool RegisterTreeNode(FrTreeNode<ParentType> *node) {
      auto path = GetPath(node);

      if (RegisterPath(path)) {
        node->SetTreePath(path);
        return true;
      }
      return false;
    }

   private:

    bool RegisterPath(const std::string &path);


    bool HasPath(const std::string &path);

    /// Gives the normalized path of the node given a hard coded policy concerning the naming scheme.
    template <class ParentType>
    static std::string GetNormalizedPathName(FrTreeNode<ParentType> *node) {

//      std::string path_name_prefix;


//      if (dynamic_cast<const FrOffshoreSystem *>(node)) {
//        path_name_prefix = "FRYDOM_";
//
//      } else if (dynamic_cast<const FrBody *>(node)) {
//        path_name_prefix = "BODY/BODY_";
//
//      } else if (dynamic_cast<const FrForce *>(node)) {
//        path_name_prefix = "FORCE/FORCE_";
//
//      } else if (dynamic_cast<const FrNode *>(node)) {
//        path_name_prefix = "NODE/NODE_";
//
//      } else if (dynamic_cast<const FrLinkBase *>(node)) { // TODO : repasser en FrLink
//        path_name_prefix = "LINK/LINK_";
//
//      //} else if (dynamic_cast<const FrActuator *>(node)) {  // TODO : a placer dans
//      //  path_name_prefix = "ACTUATORS/ACTUATOR_";           // TODO : LINKS/LINK_{name}/ACTUATOR_
//      //                                                      // TODO :
//      } else {
//        std::cerr << "No known policy for building normalized path name of " << typeid(node).name() << std::endl;
//        exit(EXIT_FAILURE);
//      }

      std::string path_name_prefix = TypeToNormalizedPathPrefix(node);


//      return path_name_prefix + node->GetName() + "/";
      return "";
    }

//   public:
//
//    template <class ParentType>
//    static std::string GetNormalizedTypeName(const FrTreeNode<ParentType> *node) {
//
//      std::string type_name;
//
//      if (dynamic_cast<const FrOffshoreSystem *>(node)) {
//        type_name = "System";
//
//      } else if (dynamic_cast<const FrBody *>(node)) {
//        type_name = "Body";
//
////      } else if (dynamic_cast<const FrForce *>(node)) {
////        type_name = "FORCE/FORCE_";
//
//      } else if (dynamic_cast<const FrNode *>(node)) {
//        type_name = "Node";
//
////      } else if (dynamic_cast<const FrLink *>(node)) {
////        type_name = "LINK/LINK_";
//
//      } else {
//        std::cerr << "No known policy for building normalized path name of " << typeid(node).name() << std::endl;
//        exit(EXIT_FAILURE);
//      }
//
//
//
//    }

   private:
    std::unordered_set<std::string> m_used_paths;

  };


} // end namespace frydom

#endif //FRYDOM_FRPATHMANAGER_H
