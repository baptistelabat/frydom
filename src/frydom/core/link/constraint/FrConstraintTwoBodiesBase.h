//
// Created by camille on 18/06/19.
//

#ifndef FRYDOM_FRCONSTRAINTTWOBODIESBASE_H
#define FRYDOM_FRCONSTRAINTTWOBODIESBASE_H

#include "chrono/solver/ChConstraintTwoBodies.h"

namespace frydom {

  namespace internal {

    /// This class override the ChConstraintTwoBodies class to be used with a full
    /// dense mass matrix
    class FrConstraintTwoBodiesBase : public chrono::ChConstraintTwoBodies {

     public:
      /// Default constructor of the class
      FrConstraintTwoBodiesBase() : chrono::ChConstraintTwoBodies() {}

      /// Copy constructor
      explicit FrConstraintTwoBodiesBase(const chrono::ChConstraintTwoBodies &source)
          : chrono::ChConstraintTwoBodies(source) {}

      /// This function updates the following auxiliary data:
      ///  - the Eq_a and Eq_b matrices
      ///  - the g_i product
      void Update_auxiliary() override;

      virtual FrConstraintTwoBodiesBase *Clone() const override { return new FrConstraintTwoBodiesBase(*this); }
    };

  } //end namespace internal

} // end namespace frydom

#endif //FRYDOM_FRCONSTRAINTTWOBODIESBASE_H
