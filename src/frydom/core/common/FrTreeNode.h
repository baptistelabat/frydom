//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRTREENODE_H
#define FRYDOM_FRTREENODE_H

#include <string>

#include "FrObject.h"


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


  template<class ParentType>
  class FrTreeNode : public FrTreeNodeBase {

   public:

    FrTreeNode(const std::string &name, ParentType *parent);

    virtual ParentType *GetParent() const;

   protected:
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
