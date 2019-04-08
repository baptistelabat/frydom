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


#ifndef FRYDOM_FRTRIANGLEMESHSHAPE_H
#define FRYDOM_FRTRIANGLEMESHSHAPE_H

#include "chrono/assets/ChTriangleMeshShape.h"

namespace frydom {
    class FrTriangleMeshConnected;

    class FrTriangleMeshShape : public chrono::ChTriangleMeshShape {
      public:
        FrTriangleMeshShape(std::shared_ptr<FrTriangleMeshConnected> mesh);
    };

}  // end namespace frydom

#endif  // FRYDOM_FRTRIANGLEMESHSHAPE_H
