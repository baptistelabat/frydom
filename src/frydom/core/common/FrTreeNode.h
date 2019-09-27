//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRTREENODE_H
#define FRYDOM_FRTREENODE_H

#include "FrObject.h"

namespace frydom {

    template<class ParentType>
    class FrTreeNode {

     public:

      FrTreeNode() = default;

      explicit FrTreeNode(ParentType *parent);

      void SetParent(ParentType *parent);

      ParentType *GetParent() const;

     protected:
      ParentType *m_parent;

         };

}  // end namespace frydom

#include "FrTreeNode.cpp"

#endif //FRYDOM_FRTREENODE_H
