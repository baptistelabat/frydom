//
// Created by frongere on 12/10/17.
//

#ifndef FRYDOM_FREASYBODIES_H
#define FRYDOM_FREASYBODIES_H

#include <chrono/physics/ChBodyEasy.h>
//#include "chrono/physics/ChMaterialSurface.h"
#include "FrBody.h"

namespace frydom {


    class FrSphere : public FrBody {
    public:
        /// Creates a ChBody plus adds a visualization shape and, optionally,
        /// a collision shape. Mass and inertia are set automatically depending
        /// on density.
        /// Sphere is assumed with center at body reference coordsystem.
        FrSphere(double radius, double mdensity, bool visual_asset = true) {

            double mmass = mdensity * ((4.0 / 3.0) * M_PI * pow(radius, 3));
            double inertia = (2.0 / 5.0) * mmass * pow(radius, 2);

            this->SetDensity((float)mdensity);
            this->SetMass(mmass);
            this->SetInertiaXX(chrono::ChVector<>(inertia, inertia, inertia));

//            if (collide) {
//                GetCollisionModel()->ClearModel();
//                GetCollisionModel()->AddSphere(radius);  // radius, radius, height on y
//                GetCollisionModel()->BuildModel();
//                SetCollide(true);
//            }
            if (visual_asset) {
                std::shared_ptr<chrono::ChSphereShape> vshape(new chrono::ChSphereShape());
                vshape->GetSphereGeometry().rad = radius;
                this->AddAsset(vshape);
            }
        }
    };





//    FrBody* MakeSphere(double radius, double density);


//    class FrSphere : public FrBody, public chrono::ChBodyEasySphere {
//
//    public:
//
//        FrSphere(double radius,
//                 double density,
//                 bool collide = false,
//                 bool visual_asset = true,
//                 chrono::ChMaterialSurface::ContactMethod contact_method = chrono::ChMaterialSurface::NSC)
//                    : ChBodyEasySphere(radius, density, collide, visual_asset, contact_method) {}
//
//    };
//
//    class FrBuoy : public FrHydroBody, public FrSphere {
//
//    public:
//
//        FrBuoy(double radius, double density) : FrSphere(radius, density) {}
//
//    };


}


#endif //FRYDOM_FREASYBODIES_H
