//
// Created by camille on 18/06/19.
//

#include "FrLinkMaskBase.h"

#include "frydom/core/link/constraint/FrConstraintTwoBodiesBase.h"

namespace frydom {

  namespace internal {

    FrLinkMaskBase::FrLinkMaskBase() {
      ResetNconstr(7);
    }

    FrLinkMaskBase::FrLinkMaskBase(int mnconstr) {
      nconstr = mnconstr;

      constraints.resize(nconstr);
      for (int i = 0; i < nconstr; i++)
        constraints[i] = new FrConstraintTwoBodiesBase;
    }

    void FrLinkMaskBase::ResetNconstr(int newnconstr) {
      int i;
      for (i = 0; i < nconstr; i++)
        if (constraints[i])
          delete constraints[i];

      nconstr = newnconstr;

      constraints.resize(nconstr);

      for (i = 0; i < nconstr; i++)
        constraints[i] = new FrConstraintTwoBodiesBase;
    }

  } // end namespace internal

} // end namespace frydom