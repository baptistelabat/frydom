//
// Created by camille on 18/06/19.
//

#ifndef FRYDOM_FRLINKMASKBASE_H
#define FRYDOM_FRLINKMASKBASE_H

#include "chrono/physics/ChLinkMask.h"

namespace frydom {

    namespace internal {

    /// Specialized ChLinkMaskLF to be used with off-diagonal mass matrix coefficients
    class FrLinkMaskBase : public chrono::ChLinkMaskLF {

    public:
        /// Create a FrLinkMaskBase with 7 scalar constraints of
        /// class FrConstraintTwoBodies.
        FrLinkMaskBase();

        /// Create a FrLinkMaskBase with nmconstr scalar constraints of
        /// class FrConstraintTwoBodies.
        FrLinkMaskBase(int mnconstr);

        /// Copy constructor
        FrLinkMaskBase(const FrLinkMaskBase& source) : chrono::ChLinkMaskLF(source) {}

        virtual FrLinkMaskBase* Clone() const override { return new FrLinkMaskBase(*this); }

        void ResetNconstr(int newnconstr) override;

    };

    } // end namespace internal

} // end namespace frydom

#endif //FRYDOM_FRLINKMASKBASE_H
