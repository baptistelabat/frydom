//
// Created by frongere on 25/09/19.
//

#include "FrTreeNode.h"


namespace frydom {

  template <class ParentType>
  void FrTreeNode<ParentType>::SetParent(ParentType *parent) {
    m_parent = parent;
  }

  template <class ParentType>
  ParentType *FrTreeNode<ParentType>::GetParent() const {
    return m_parent;
  }

}  // end namespace frydom
