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


#ifndef FRYDOM_FRCYLINDERSHAPE_H
#define FRYDOM_FRCYLINDERSHAPE_H

#include <memory>
#include "frydom/asset/FrAssetOwner.h"

namespace chrono {
    class ChAsset;
    class ChCylinderShape;
}  // end namespace chrono


namespace frydom {

    class FrCylinderShape {
      public:
        FrCylinderShape(double radius, double height);
        double radius() const;
        double height() const;

      protected:
        std::shared_ptr<chrono::ChAsset> GetChronoAsset();

      private:
          friend void FrAssetOwner::AddCylinderShape(double, double);
          std::shared_ptr<chrono::ChCylinderShape> m_cylinder;
    };

}  // end namespace frydom

#endif  // FRYDOM_FRCYLINDERSHAPE_H
