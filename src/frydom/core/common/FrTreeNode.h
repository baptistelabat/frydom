//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRTREENODE_H
#define FRYDOM_FRTREENODE_H

#include <string>

//#include "frydom/logging/FrPathManager.h"

#include "FrObject.h"


namespace frydom {

  class FrTreeNodeBase {

   public:

    explicit FrTreeNodeBase(const std::string &name) : m_name(name) {}

    virtual ~FrTreeNodeBase() = default; // To make the class polymorphic

    const std::string &GetName() const {
      return m_name;
    }

    void SetTreePath(const std::string &tree_path) {
      m_tree_path = tree_path;
    }

   private:
    std::string m_name;

   protected:
    std::string m_tree_path; // Non utilise !!

  };

  class FrOffshoreSystem;


  template<class ParentType>
  class FrTreeNode : public FrTreeNodeBase {

   public:

    /// Constructor
    FrTreeNode(const std::string &name, ParentType *parent);

    /// Get a pointer to the Parent TreeNode
    virtual ParentType *GetParent() const;

    FrOffshoreSystem* GetSystem();

   private:
    ParentType *m_parent = nullptr;

  };

  class FrRootNode : public FrTreeNode<FrRootNode> {

   public:
    FrRootNode *GetParent() const final {
      return nullptr;
    }

  };


}  // end namespace frydom

#include "FrTreeNode.cpp"

#endif //FRYDOM_FRTREENODE_H
