//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRTREENODE_H
#define FRYDOM_FRTREENODE_H

#include <string>

#include "FrObject.h"


//#include "frydom/core/FrOffshoreSystem.h"


namespace frydom {

  class FrTreeNodeBase {

   public:

    explicit FrTreeNodeBase(const std::string &name) : m_name(name) {}

    const std::string &GetName() const {
      return m_name;
    }

   private:
    std::string m_name;

  };

  class FrOffshoreSystem;


  template<class ParentType>
  class FrTreeNode : public FrTreeNodeBase {

   public:

    /// Constructor
    FrTreeNode(const std::string &name, ParentType *parent);

    /// Get a pointer to the Parent TreeNode
    virtual ParentType *GetParent() const;

//    /// Returns a pointer to the root FrOffshoreSystem for any TreeNode by a recursive run time procedure.
//    FrOffshoreSystem *GetSystem() {
//
//      if (dynamic_cast<FrOffshoreSystem *>(this)) {
//        return dynamic_cast<FrOffshoreSystem * >(this);
//      } else {
//        return m_parent->GetSystem();
//      }
//
//    }
//
//    const FrOffshoreSystem *GetSystem() const {
//
//      if (dynamic_cast<const FrOffshoreSystem *>(this)) {
//        return dynamic_cast<const FrOffshoreSystem * >(this);
//      } else {
//        return m_parent->GetSystem();
//      }
//
//    }

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
