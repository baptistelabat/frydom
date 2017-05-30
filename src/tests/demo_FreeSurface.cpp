// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// demo code for free surface definition
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "../core/FrOffshoreSystem.h"

#include <irrlicht.h>

int main(int argc, char* argv[]) {

    // Creating the system
    frydom::FrOffshoreSystem system;

//    auto ptr = system.shared_from_this();

//    auto sys_ptr = system.getShared();

    // Getting the default free surface
//    std::shared_ptr<frydom::environment::FrFreeSurface> fs = system.getFreeSurface();

    // Changing the default free surface
    // TODO !!

//    fs->Initialize(0, 10, 1);
//
//    std::cout << fs->getMesh().getNumTriangles();
//    fs->getMesh().Clear();
//    std::cout << fs->getMesh().getNumTriangles();

    // Creating the free surface
//    frydom::environment::FrFlatFreeSurface free_surface(&system, 2);
//    free_surface.Initialize(0, 100, 1);


    // Trying to view it into irrlicht
//    chrono::irrlicht::ChIrrApp app(&system, L"Visu free surface", irr::core::dimension2d<irr::u32>(800, 600), false, true);


    // Converting into Irrlicht meshes the assets that have been added
//    app.AssetUpdate()






    return 0;

}

