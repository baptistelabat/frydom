//
// Created by Lucas Letournel on 03/08/18.
//

#ifndef FRYDOM_FRASSETCLUMPWEIGHT_H
#define FRYDOM_FRASSETCLUMPWEIGHT_H

#include <chrono/physics/ChBodyEasy.h>

namespace frydom {
    class FrAssetClumpWeight : public chrono::ChCylinderShape{
    public:
        FrAssetClumpWeight():chrono::ChCylinderShape(){
            SetRadius(1); SetColor(chrono::ChColor(1,1,0,1)); SetAxis(chrono::ChVector<>(0,0,-1),chrono::ChVector<>(0,0,1));
        };

        FrAssetClumpWeight(double mradius):chrono::ChCylinderShape(){
            SetRadius(mradius); SetColor(chrono::ChColor(1,1,0,1)); SetAxis(chrono::ChVector<>(0,0,0),chrono::ChVector<>(0,0,1));
        };

        FrAssetClumpWeight(double mradius,chrono::ChColor mcolor):chrono::ChCylinderShape(){
            SetRadius(mradius); SetColor(mcolor); SetAxis(chrono::ChVector<>(0,0,0),chrono::ChVector<>(0,0,1));
        };

        FrAssetClumpWeight(double mradius,chrono::ChColor mcolor,chrono::ChVector<> P1, chrono::ChVector<> P2)
                :chrono::ChCylinderShape(){SetRadius(mradius); SetColor(mcolor); SetAxis(P1,P2);};


        void SetCylinderGeometry(chrono::geometry::ChCylinder mcylinder) {gcylinder = mcylinder;}
        void SetRadius(double mradius) {GetCylinderGeometry().rad = mradius;}
        void SetAxis(chrono::ChVector<> P1, chrono::ChVector<> P2) {GetCylinderGeometry().p1 = P1; GetCylinderGeometry().p2 = P2;}



    };
}
#endif //FRYDOM_FRASSETCLUMPWEIGHT_H
