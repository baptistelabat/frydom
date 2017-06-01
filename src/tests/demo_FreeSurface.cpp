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
#include "../environment/waves/FrFlatFreeSurface.h"

#include <irrlicht.h>

int main(int argc, char* argv[]) {

    // Creating the system
    auto system = frydom::FrOffshoreSystem();

    // Creating the free surface and assigning it to a unique pointer as we should have only one free surface
    auto free_surface = std::make_unique<frydom::environment::FrFlatFreeSurface>(2.);
    free_surface->Initialize(0, 50, 0.5);

    // Giving the free surface's ownership to the system (it becomes responsible of the destruction)
    system.setFreeSurface(free_surface.release()); // le release effectue un transfert de propriete de free_surface a system qui devient responsable de la destruction


    // Trying to view it into irrlicht
    chrono::irrlicht::ChIrrApp app(&system, L"Visu free surface", irr::core::dimension2d<irr::u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, sky in Irrlicht scene
    app.AddTypicalSky("../frydom/core/chrono/build/data/skybox/");
    app.AddTypicalLights();
    app.AddTypicalCamera();
//    app.AddTypicalCamera(irr::core::vector3df(0, 4, -6));



    app.AssetBindAll();

    app.AssetUpdateAll();

    app.SetStepManage(true);
    app.SetTimestep(0.01);
    app.SetTryRealtime(true);

    while (app.GetDevice()->run()) {
        app.BeginScene();
        app.DrawAll();
        app.DoStep();
        app.EndScene();
    }




    std::cout << "LEAVING MAIN PROGRAM" << "\n";
    return 0;

}
