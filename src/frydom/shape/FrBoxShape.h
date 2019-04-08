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

#include "chrono/assets/ChBoxShape.h"

namespace frydom {

    class FrBoxShape : public chrono::ChBoxShape {
      public:
        FrBoxShape(double xSize, double ySize, double zSize);
    };

}  // end namespace frydom

#endif  // FRYDOM_FRBOXSHAPE_H
