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


// TODO : renommer en FrTreeNode.hxx
namespace frydom {

  template<class ParentType>
  FrTreeNode<ParentType>::FrTreeNode(const std::string &name, ParentType *parent) :
      FrTreeNodeBase(name),
      m_parent(parent) {}

  template<class ParentType>
  ParentType *FrTreeNode<ParentType>::GetParent() const {
    return m_parent;
  }

  template<class ParentType>
  FrOffshoreSystem *FrTreeNode<ParentType>::GetSystem() {
    if (auto system = dynamic_cast<FrOffshoreSystem *>(this)) {
      return system;
    } else {
      return m_parent->GetSystem();
    }
  }

  template<class ParentType>
  FrOffshoreSystem *FrTreeNode<ParentType>::GetSystem() const {
    return GetSystem();
  }

}  // end namespace frydom
