//
// Created by Lucas Letournel on 03/08/18.
//

#ifndef FRYDOM_FRASSETBUOY_H
#define FRYDOM_FRASSETBUOY_H

#include <chrono/physics/ChBodyEasy.h>

namespace frydom {
    class FrAssetBuoy : public chrono::ChSphereShape {
    private:
    public:
        FrAssetBuoy() :ChSphereShape(){SetRadius(1); SetColor(chrono::ChColor(1,0,0,1));}
        //FrAssetBuoy(const chrono::geometry::ChSphere msphere) :ChSphereShape(const chrono::geometry::ChSphere& msphere){};
        FrAssetBuoy(double radius) :ChSphereShape(){SetRadius(radius); SetColor(chrono::ChColor(1,0,0,1));}
        FrAssetBuoy(double mradius, chrono::ChColor mcolor) :ChSphereShape(){SetRadius(mradius); SetColor(mcolor);}

        void SetSphereGeometry(chrono::geometry::ChSphere SphereGeom) {gsphere = SphereGeom;}
        void SetRadius(double mradius) {GetSphereGeometry().rad = mradius;}
        double GetRadius() {return GetSphereGeometry().rad;}
    };

}
#endif //FRYDOM_FRASSETBUOY_H
