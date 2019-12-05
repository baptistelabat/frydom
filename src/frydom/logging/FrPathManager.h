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

    template<class NodeType>
    static std::string GetPath(NodeType *node) {
      if (!node) return "";

      return GetPath(node->GetParent()) + GetNormalizedPathName(node);
    }

    template<class NodeType>
    static std::string GetPath(const NodeType &node) {
      return GetPath(&node);
    }


    template<class NodeType>
    void RegisterTreeNode(NodeType *node) {
      auto path = GetPath(node);

      if (!RegisterPath(path)) {

        throw std::runtime_error("Object with name " + node->GetName() + " already exists in this context. Defined twice.");
      }

//      if (RegisterPath(path)) {
        node->SetTreePath(path);
//        return true;
//      }
//      return false;
    }

   private:

    bool RegisterPath(const std::string &path);


    bool HasPath(const std::string &path);

    /// Gives the normalized path of the node given a hard coded policy concerning the naming scheme.
    template <class NodeType>
    static std::string GetNormalizedPathName(NodeType *node) {
      return TypeToNormalizedPathPrefix(node) + node->GetName() + "/";
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
