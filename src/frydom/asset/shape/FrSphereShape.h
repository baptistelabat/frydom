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


#ifndef FRYDOM_FRSPHERESHAPE_H
#define FRYDOM_FRSPHERESHAPE_H

#include <memory>
#include "frydom/asset/FrAssetOwner.h"

namespace chrono {
  class ChAsset;

  class ChSphereShape;
}  // end namespace chrono

namespace frydom {

  class FrSphereShape {

   public:
    FrSphereShape(double radius, const Position& relative_position, FRAME_CONVENTION fc);

    double radius() const;

   protected:
    std::shared_ptr<chrono::ChAsset> GetChronoAsset();

   private:
    friend void FrAssetOwner::AddSphereShape(double, const Position&, FRAME_CONVENTION);

    std::shared_ptr<chrono::ChSphereShape> m_sphere;
  };

}  // end namespace frydom

#endif  // FRYDOM_FRSPHERESHAPE_H
