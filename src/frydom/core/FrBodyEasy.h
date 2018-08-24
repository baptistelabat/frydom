//
// Created by frongere on 12/10/17.
//

#ifndef FRYDOM_FREASYBODIES_H
#define FRYDOM_FREASYBODIES_H


#include <chrono/physics/ChBodyEasy.h>
//#include "chrono/physics/ChMaterialSurface.h"
#include "FrBody.h"
#include "FrHydroBody.h"

namespace frydom {


    class FrSphere : public FrBody {
    public:
        /// Creates a ChBody plus adds a visualization shape and, optionally,
        /// a collision shape. Mass and inertia are set automatically depending
        /// on density.
        /// Sphere is assumed with center at body reference coordsystem.
        FrSphere(double radius, double mass, bool visual_asset = true) {

//            double mmass = mdensity * ((4.0 / 3.0) * M_PI * pow(radius, 3));
            double inertia = (2.0 / 5.0) * mass * pow(radius, 2);

//            this->SetDensity((float)mdensity);
            this->SetMass(mass);
            this->SetInertiaXX(chrono::ChVector<>(inertia, inertia, inertia));

            // TODO: remettre en place les modeles de collision.
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


    class FrBox : public FrBody {

    public:
        FrBox(double Xsize, double Ysize, double Zsize, double mass, bool collide=false, bool visual_asset=true) : FrBody() {

            matsurface = std::make_shared<chrono::ChMaterialSurfaceSMC>();

            this->SetMass(mass);
            this->SetInertiaXX(chrono::ChVector<>((1.0 / 12.0) * mass * (pow(Ysize, 2) + pow(Zsize, 2)),
                                                  (1.0 / 12.0) * mass * (pow(Xsize, 2) + pow(Zsize, 2)),
                                                  (1.0 / 12.0) * mass * (pow(Xsize, 2) + pow(Ysize, 2))));

            if (collide) {
                GetCollisionModel()->ClearModel();
                GetCollisionModel()->AddBox(Xsize * 0.5, Ysize * 0.5, Zsize * 0.5);  // radius x, radius z, height on y
                GetCollisionModel()->BuildModel();
                SetCollide(true);
            }

            if (visual_asset) {
                auto vshape = std::make_shared<chrono::ChBoxShape>();
                vshape->GetBoxGeometry().SetLengths(chrono::ChVector<>(Xsize, Ysize, Zsize));
                this->AddAsset(vshape);
            }

        }
    };


    // TODO: creer un FrBuoy qui gere automatiquement des modeles hydro et qui derive de FrSphere
    class FrCylinder : public FrBody {

    public:
        FrCylinder(double radius, double height, double mass, bool collide, bool visual_asset) : FrBody() {

            matsurface = std::make_shared<chrono::ChMaterialSurfaceSMC>();

            this->SetMass(mass);
            this->SetInertiaXX(chrono::ChVector<>((1.0 / 12.0) * mass * (3 * pow(radius, 2) + pow(height, 2)),
                                                  0.5 * mass * pow(radius, 2),
                                                  (1.0 / 12.0) * mass * (3 * pow(radius, 2) + pow(height, 2))));

            if (collide) {
                GetCollisionModel()->ClearModel();
                GetCollisionModel()->AddCylinder(radius, radius, height * 0.5);  // radius x, radius z, height on y
                GetCollisionModel()->BuildModel();
                SetCollide(true);
            }
            if (visual_asset) {
                auto vshape = std::make_shared<chrono::ChCylinderShape>();
                vshape->GetCylinderGeometry().p1 = chrono::ChVector<>(0, -height * 0.5, 0);
                vshape->GetCylinderGeometry().p2 = chrono::ChVector<>(0, height * 0.5, 0);
                vshape->GetCylinderGeometry().rad = radius;
                this->AddAsset(vshape);
            }

        }




    };





}


#endif //FRYDOM_FREASYBODIES_H
