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


#include "FrAssetClumpWeight.h"

#include "chrono/geometry/ChCylinder.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"


namespace frydom {


    FrAssetClumpWeight::FrAssetClumpWeight(double mradius, chrono::ChColor mcolor, chrono::ChVector<> P1,
                                           chrono::ChVector<> P2) {
        chrono::geometry::ChCylinder mcylinder(P1,P2,mradius);
        m_shape= std::make_shared<chrono::ChCylinderShape>(mcylinder);
        m_color = std::make_shared<chrono::ChColorAsset>(mcolor);
    }

    FrAssetClumpWeight::FrAssetClumpWeight(double mradius, chrono::ChVector<> P1, chrono::ChVector<> P2)
            :FrAssetClumpWeight(mradius,chrono::ChColor(0.6f,0.6f,0.f), P1, P2){}

    FrAssetClumpWeight::FrAssetClumpWeight(double mradius)
            :FrAssetClumpWeight(mradius,chrono::ChColor(0.6f,0.6f,0.f), chrono::ChVector<>(0,0,-1), chrono::ChVector<>(0,0,1)){}

    FrAssetClumpWeight::FrAssetClumpWeight()
            :FrAssetClumpWeight(1,chrono::ChColor(0.6f,0.6f,0.f), chrono::ChVector<>(0,0,-1), chrono::ChVector<>(0,0,1)){}


}  // end namespace frydom
