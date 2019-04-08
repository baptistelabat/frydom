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
#ifndef FRYDOM_FRBOXSHAPE_H
#define FRYDOM_FRBOXSHAPE_H

#include <memory>
#include "frydom/asset/FrAssetOwner.h"

namespace chrono {
    class ChBoxShape;
    class ChAsset;
}  // end namespace chrono

namespace frydom {

    class FrBoxShape {
      public:
        FrBoxShape(double xSize, double ySize, double zSize);

      protected:
        std::shared_ptr<chrono::ChAsset> GetChronoAsset();

      private:
        friend void FrAssetOwner::AddBoxShape(double, double, double);
        std::shared_ptr<chrono::ChBoxShape> m_box;
    };

}  // end namespace frydom

#endif  // FRYDOM_FRBOXSHAPE_H
