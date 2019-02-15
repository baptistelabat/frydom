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


#ifndef FRYDOM_FRASSETBUOY_H
#define FRYDOM_FRASSETBUOY_H

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChColorAsset.h"
#include "FrAssetComponent.h"

namespace frydom {
/*

    class FrAssetBuoy : public chrono::ChSphereShape {
    public:

        FrAssetBuoy(double radius) :chrono::ChSphereShape() {
            GetSphereGeometry().rad = radius;
        }


    };

}
*/

    /**
     * \class FrAssetBuoy
     * \brief Class for the buoy assets.
     */
    class FrAssetBuoy : public FrAssetComponent {
    private:
        //std::shared_ptr<chrono::ChColorAsset> m_color;
        //std::shared_ptr<chrono::ChSphereShape> m_shape;
    public:

        FrAssetBuoy(chrono::ChVector<> mPos, double mRadius, chrono::ChColor mColor){
            chrono::geometry::ChSphere msphere(mPos, mRadius);
            m_shape = std::make_shared<chrono::ChSphereShape>(msphere);
            m_color = std::make_shared<chrono::ChColorAsset>(mColor);
        }
        FrAssetBuoy(double mRadius, chrono::ChColor mColor) :FrAssetBuoy(chrono::ChVector<>(0,0,0), mRadius, mColor){}

        FrAssetBuoy(double mRadius) :FrAssetBuoy(chrono::ChVector<>(0,0,0), mRadius, chrono::ChColor(1.f, 0.f, 0.0f)){}

        FrAssetBuoy() :FrAssetBuoy(chrono::ChVector<>(0,0,0), 1, chrono::ChColor(1.f, 0.f, 0.0f)){}


        //std::shared_ptr<chrono::ChColorAsset> GetColorAsset() {return m_color;}
        //std::shared_ptr<chrono::ChSphereShape> GetShapeAsset() {return m_shape;}

        void SetColorAsset(std::shared_ptr<chrono::ChColorAsset> color) {m_color = color;}
        void SetShapeAsset(std::shared_ptr<chrono::ChSphereShape> shape) {m_shape = shape;}

        /*
        std::shared_ptr<chrono::ChVisualization> GetShapeAsset() override {return m_shape;}
        void SetRadius(double mradius) { dynamic_cast<chrono::ChSphereShape*>(m_shape)->GetSphereGeometry().rad = mradius;}
        double GetRadius() {return dynamic_cast<chrono::ChSphereShape*>(m_shape)->GetSphereGeometry().rad;}
         */
    };

} // end namespace frydom
#endif //FRYDOM_FRASSETBUOY_H
