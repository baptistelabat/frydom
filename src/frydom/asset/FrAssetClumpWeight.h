// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRASSETCLUMPWEIGHT_H
#define FRYDOM_FRASSETCLUMPWEIGHT_H

#include <chrono/physics/ChBodyEasy.h>

namespace frydom {

    /**
     * \class FrAssetClumpWeight
     * \brief Class for the clump weight assets.
     */
    class FrAssetClumpWeight : public FrAssetComponent{

    public:
        FrAssetClumpWeight(double mradius,chrono::ChColor mcolor,chrono::ChVector<> P1, chrono::ChVector<> P2){
            chrono::geometry::ChCylinder mcylinder(P1,P2,mradius);
            m_shape= std::make_shared<chrono::ChCylinderShape>(mcylinder);
            m_color = std::make_shared<chrono::ChColorAsset>(mcolor);
        }

        FrAssetClumpWeight(double mradius,chrono::ChVector<> P1, chrono::ChVector<> P2)
            :FrAssetClumpWeight(mradius,chrono::ChColor(0.6f,0.6f,0.f), P1, P2){}

        FrAssetClumpWeight(double mradius)
            :FrAssetClumpWeight(mradius,chrono::ChColor(0.6f,0.6f,0.f), chrono::ChVector<>(0,0,-1), chrono::ChVector<>(0,0,1)){}

        FrAssetClumpWeight()
            :FrAssetClumpWeight(1,chrono::ChColor(0.6f,0.6f,0.f), chrono::ChVector<>(0,0,-1), chrono::ChVector<>(0,0,1)){}

        //void SetCylinderGeometry(chrono::geometry::ChCylinder mcylinder) {gcylinder = mcylinder;}
        //void SetRadius(double mradius) {dynamic_cast<chrono::ChCylinderShape*>(m_shape)-> GetCylinderGeometry().rad = mradius;}
        /*void SetAxis(chrono::ChVector<> P1, chrono::ChVector<> P2) {
            dynamic_cast<chrono::ChCylinderShape*>(m_shape)-> GetCylinderGeometry().p1 = P1;
            dynamic_cast<chrono::ChCylinderShape*>(m_shape)-> GetCylinderGeometry().p2 = P2;
        }*/
        //std::shared_ptr<chrono::ChVisualization> GetShapeAsset() override {return m_shape;}



    };
}
#endif //FRYDOM_FRASSETCLUMPWEIGHT_H
