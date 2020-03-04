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


#ifndef FRYDOM_FRTREENODE_H
#define FRYDOM_FRTREENODE_H

#include <string>

#include "FrObject.h"

namespace frydom {

  class FrTreeNodeBase {

   public:

    explicit FrTreeNodeBase(const std::string &name) : m_name(name) {}

    virtual ~FrTreeNodeBase() = default; // To make the class polymorphic we need at least one virtual method...

    const std::string &GetName() const {
      return m_name;
    }

    void SetTreePath(const std::string &tree_path) {
      m_tree_path = tree_path;
    }

    const std::string &GetTreePath() const {
      return m_tree_path;
    }

   private:
    std::string m_name;
    std::string m_tree_path;

  };

  // Forward declaration
  class FrOffshoreSystem;

  template<class ParentType>
  class FrTreeNode : public FrTreeNodeBase {

   public:

    /// Constructor
    FrTreeNode(const std::string &name, ParentType *parent);

    /// Get a pointer to the Parent TreeNode
    virtual ParentType *GetParent() const;

    FrOffshoreSystem *GetSystem();

    FrOffshoreSystem *GetSystem() const;

   private:
    ParentType *m_parent;

  };

  class FrRootNode : public FrTreeNode<FrRootNode> {

   public:
    FrRootNode *GetParent() const final {
      std::cout << "test" << std::endl;
      return nullptr;
    }

  };

}  // end namespace frydom

#include "FrTreeNode.cpp"

#endif //FRYDOM_FRTREENODE_H
