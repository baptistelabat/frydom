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

#include <memory>
#include "frydom/asset/FrAssetOwner.h"

namespace chrono {
    class ChAsset;
    class ChTriangleMeshShape;
}  // end namespace chrono


namespace frydom {
    class FrTriangleMeshConnected;

    class FrTriangleMeshShape {
      public:
        FrTriangleMeshShape(std::shared_ptr<FrTriangleMeshConnected> mesh);

      protected:
        std::shared_ptr<chrono::ChAsset> GetChronoAsset();

      private:
        friend void FrAssetOwner::AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected>);
        std::shared_ptr<chrono::ChTriangleMeshShape> m_mesh;
    };

}  // end namespace frydom

#endif  // FRYDOM_FRTRIANGLEMESHSHAPE_H
