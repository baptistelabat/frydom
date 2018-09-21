//
// Created by frongere on 12/10/17.
//

#include "FrBodyEasy.h"

#include "FrBody.h"

#include "chrono/physics/ChBodyEasy.h"


namespace frydom {



    std::shared_ptr<FrBody_> make_BoxBody(double xSize, double ySize, double zSize, double mass) {

        double volume = xSize * ySize * zSize;
        double density = mass / volume;

        auto box = std::make_shared<FrBody_>();

        auto chronoBody = std::dynamic_pointer_cast<_FrBodyBase>(
                std::make_shared<chrono::ChBodyEasyBox>(xSize, ySize, zSize, density, true, true, chrono::ChMaterialSurface::SMC)
                );

        box->m_chronoBody = chronoBody;
        box->SetSmoothContact();

        return box;
    }

    std::shared_ptr<FrBody_> make_CylinderBody(double radius, double height, double mass) {
        double volume = MU_PI * radius * radius * height;
        double density = mass / volume;

        auto cylinder = std::make_shared<FrBody_>();

        auto chronoBody = std::dynamic_pointer_cast<_FrBodyBase>(
                std::make_shared<chrono::ChBodyEasyCylinder>(radius, height, density, true, true, chrono::ChMaterialSurface::SMC)
        );

        cylinder->m_chronoBody = chronoBody;
        cylinder->SetSmoothContact();

        return cylinder;
    }

    std::shared_ptr<FrBody_> make_SphereBody(double radius, double mass) {
        double volume = (4. / 3.) * MU_PI * radius * radius * radius;
        double density = mass / volume;

        auto sphere = std::make_shared<FrBody_>();

        auto chronoBody = std::dynamic_pointer_cast<_FrBodyBase>(
                std::make_shared<chrono::ChBodyEasyCylinder>(radius, density, true, true, chrono::ChMaterialSurface::SMC)
        );

        sphere->m_chronoBody = chronoBody;
        sphere->SetSmoothContact();

        return sphere;
    }






}