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


#include "FrAssetBuoy.h"


namespace frydom {


    FrAssetBuoy::FrAssetBuoy(chrono::ChVector<> mPos, double mRadius, chrono::ChColor mColor) {
        chrono::geometry::ChSphere msphere(mPos, mRadius);
        m_shape = std::make_shared<chrono::ChSphereShape>(msphere);
        m_color = std::make_shared<chrono::ChColorAsset>(mColor);
    }

    FrAssetBuoy::FrAssetBuoy(double mRadius, chrono::ChColor mColor) :FrAssetBuoy(chrono::ChVector<>(0,0,0), mRadius, mColor){}

    FrAssetBuoy::FrAssetBuoy(double mRadius) :FrAssetBuoy(chrono::ChVector<>(0,0,0), mRadius, chrono::ChColor(1.f, 0.f, 0.0f)) {}

    FrAssetBuoy::FrAssetBuoy() :FrAssetBuoy(chrono::ChVector<>(0,0,0), 1, chrono::ChColor(1.f, 0.f, 0.0f)){}

    void FrAssetBuoy::SetColorAsset(std::shared_ptr<chrono::ChColorAsset> color) {m_color = color;}

    void FrAssetBuoy::SetShapeAsset(std::shared_ptr<chrono::ChSphereShape> shape) {m_shape = shape;}
}  // end namespace frydom
