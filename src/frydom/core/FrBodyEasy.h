//
// Created by frongere on 12/10/17.
//

#ifndef FRYDOM_FREASYBODIES_H
#define FRYDOM_FREASYBODIES_H

#include <memory>

//#include <chrono/physics/ChBodyEasy.h>
////#include "chrono/physics/ChMaterialSurface.h"
//#include "FrBody.h"
//#include "FrHydroBody.h"

namespace frydom {


//    class FrSphere : public FrBody {
//    public:
//        /// Creates a ChBody plus adds a visualization shape and, optionally,
//        /// a collision shape. Mass and inertia are set automatically depending
//        /// on density.
//        /// Sphere is assumed with center at body reference coordsystem.
//        FrSphere(double radius, double mass, bool visual_asset = true) {
//
////            double mmass = mdensity * ((4.0 / 3.0) * M_PI * pow(radius, 3));
//            double inertia = (2.0 / 5.0) * mass * pow(radius, 2);
//
////            this->SetDensity((float)mdensity);
//            this->SetMass(mass);
//            this->SetInertiaXX(chrono::ChVector<>(inertia, inertia, inertia));
//
//            // TODO: remettre en place les modeles de collision.
////            if (collide) {
////                GetCollisionModel()->ClearModel();
////                GetCollisionModel()->AddSphere(radius);  // radius, radius, height on y
////                GetCollisionModel()->BuildModel();
////                SetCollide(true);
////            }
//            if (visual_asset) {
//                std::shared_ptr<chrono::ChSphereShape> vshape(new chrono::ChSphereShape());
//                vshape->GetSphereGeometry().rad = radius;
//                this->AddAsset(vshape);
//            }
//        }
//    };

//    // TODO: creer un FrBuoy qui gere automatiquement des modeles hydro et qui derive de FrSphere
//    class FrHydroCylinder : public FrHydroBody {
//
//
//    };









    /// REFACTORING ----------->>>>>

    // Forward declaration
    class FrBody_;

    void makeItBox(std::shared_ptr<FrBody_> body, double xSize, double ySize, double zSize, double mass);

    void makeItCylinder(std::shared_ptr<FrBody_> body, double radius, double height, double mass);

    void makeItSphere(std::shared_ptr<FrBody_> body, double radius, double mass);


    std::shared_ptr<FrBody_> make_BoxBody(double xSize, double ySize, double zSize, double mass);

    std::shared_ptr<FrBody_> make_CylinderBody(double radius, double height, double mass);

    std::shared_ptr<FrBody_> make_SphereBody(double radius, double mass);



}  // end namespace frydom


#endif //FRYDOM_FREASYBODIES_H
