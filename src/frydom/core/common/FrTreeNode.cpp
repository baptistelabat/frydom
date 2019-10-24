//
// Created by frongere on 25/09/19.
//


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
      return GetParent()->GetSystem();
    }
  }

}  // end namespace frydom
