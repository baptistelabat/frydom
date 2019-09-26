//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRTREENODE_H
#define FRYDOM_FRTREENODE_H

#include "FrObject.h"

namespace frydom {

  template <class ParentType>
  class FrTreeNode {

   protected:

    void SetParent(ParentType* parent);

    ParentType* GetParent() const;

   private:
    ParentType* m_parent;

  };

}  // end namespace frydomœ


#endif //FRYDOM_FRTREENODE_H
