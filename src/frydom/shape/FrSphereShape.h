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

#include "chrono/assets/ChSphereShape.h"

namespace frydom {

 class FrSphereShape : public chrono::ChSphereShape {
      public:
        FrSphereShape(double radius);
    };

}  // end namespace frydom

#endif  // FRYDOM_FRSPHERESHAPE_H
