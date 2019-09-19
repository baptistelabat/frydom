//
// Created by camille on 18/06/19.
//

#include "FrLinkMaskBase.h"

#include "frydom/core/link/constraint/FrConstraintTwoBodiesBase.h"

namespace frydom {

    namespace internal {

        template<typename OffshoreSystemType>
        FrLinkMaskBase<OffshoreSystemType>::FrLinkMaskBase() {
          ResetNconstr(7);
        }

        template<typename OffshoreSystemType>
        FrLinkMaskBase<OffshoreSystemType>::FrLinkMaskBase(int mnconstr) {
          nconstr = mnconstr;

          constraints.resize(nconstr);
          for (int i = 0; i < nconstr; i++)
            constraints[i] = new FrConstraintTwoBodiesBase<OffshoreSystemType>;
        }

        template<typename OffshoreSystemType>
        void FrLinkMaskBase<OffshoreSystemType>::ResetNconstr(int newnconstr) {
          int i;
          for (i = 0; i < nconstr; i++)
            if (constraints[i])
              delete constraints[i];

          nconstr = newnconstr;

          constraints.resize(nconstr);

          for (i = 0; i < nconstr; i++)
            constraints[i] = new FrConstraintTwoBodiesBase<OffshoreSystemType>;
        }

    } // end namespace internal

} // end namespace frydom
