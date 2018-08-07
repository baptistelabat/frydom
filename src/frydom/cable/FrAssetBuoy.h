//
// Created by Lucas Letournel on 03/08/18.
//

#ifndef FRYDOM_FRASSETBUOY_H
#define FRYDOM_FRASSETBUOY_H

#include <chrono/physics/ChBodyEasy.h>
#include "frydom/frydom.h"

namespace frydom {
    class FrAssetBuoy : public chrono::ChSphereShape {
    public:

        FrAssetBuoy(double radius) :chrono::ChSphereShape() {
            GetSphereGeometry().rad = radius;
        }


    };

}

/*
    class FrAssetBuoy : public FrAssetComponent {

        //chrono::ChSphereShape
    private:
    public:

        FrAssetBuoy(chrono::ChVector<> mPos, double mRadius, chrono::ChColor mColor){
            chrono::geometry::ChSphere msphere(mPos, mRadius);
            m_shape = std::make_shared<chrono::ChSphereShape>(msphere);
            m_color = std::make_shared<chrono::ChColorAsset>(mColor);
        }
        FrAssetBuoy(double mRadius, chrono::ChColor mColor){
            FrAssetBuoy(chrono::ChVector<>(0,0,0), mRadius, mColor);
        }
        FrAssetBuoy(double mRadius){
            FrAssetBuoy(chrono::ChVector<>(0,0,0), mRadius, chrono::ChColor(1.f, 0.f, 0.0f));
        }
        FrAssetBuoy(){
            FrAssetBuoy(chrono::ChVector<>(0,0,0), 1, chrono::ChColor(1.f, 0.f, 0.0f));
        }

        //std::shared_ptr<chrono::ChVisualization> GetShapeAsset() override {return m_shape;}
        *//*void SetRadius(double mradius) { dynamic_cast<chrono::ChSphereShape*>(m_shape)->GetSphereGeometry().rad = mradius;}
        double GetRadius() {return dynamic_cast<chrono::ChSphereShape*>(m_shape)->GetSphereGeometry().rad;}*//*
    };

}*/
#endif //FRYDOM_FRASSETBUOY_H
