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

#include "chrono/assets/ChCylinderShape.h"

namespace frydom {

    class FrCylinderShape : public chrono::ChCylinderShape {
      public:
        FrCylinderShape(double radius, double height);
    };

}  // end namespace frydom

#endif  // FRYDOM_FRCYLINDERSHAPE_H
