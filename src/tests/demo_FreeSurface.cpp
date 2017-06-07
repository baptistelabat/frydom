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

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "../core/FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"
#include "../utils/FrIrrApp.h"
#include "../hydrodynamics/FrTryalForce.h"

#include <irrlicht.h>

bool viz = true;
float friction = 0.6f;
double step_size = 1e-2;



int main(int argc, char* argv[]) {

    // Creating the system
    auto system = frydom::FrOffshoreSystem();

    // Creating the free surface and assigning it to a unique pointer as we should have only one free surface that has to be owned by the OffshoreSystem
    auto free_surface = std::make_unique<frydom::environment::FrFlatFreeSurface>(0.);
    free_surface->Initialize(-100, 100, 25, -30, 30, 15);

    // Giving the free surface's ownership to the system (it becomes responsible of the destruction)
    system.setFreeSurface(free_surface.release()); // le release effectue un transfert de propriete de free_surface a system qui devient responsable de la destruction


    // Contact method
//    chrono::ChMaterialSurface::ContactMethod contactMethod = chrono::ChMaterialSurface::SMC;


    // Creating a body that has to be a floating body
    auto ship = std::make_shared<chrono::ChBody>();
    ship->SetName("my_ship");
//    auto ship = std::make_shared<chrono::ChBody>(chrono::ChMaterialSurface::NSC);
    ship->SetIdentifier(1);
    ship->SetMass(5e6);
    ship->SetPos(chrono::ChVector<>(0, 0, 0));
    ship->SetRot(chrono::ChQuaternion<>(1, 0, 0, 0));

    ship->SetPos_dt(chrono::ChVector<>(10, 0, 30));
    ship->SetRot_dt(chrono::ChQuaternion<>(0.3, chrono::ChVector<>(0, 1, 0)));


    ship->SetBodyFixed(false); // TODO: debloquer
//    auto material = std::make_shared<chrono::ChMaterialSurfaceNSC>();
//    ship->SetMaterialSurface(material);auto

    ship->SetInertiaXX(chrono::ChVector<>(1e5, 5e6, 5e6));

    // Definig the shape
    auto mesh = chrono::geometry::ChTriangleMeshConnected();
//    mesh.RepairDuplicateVertexes();
    mesh.LoadWavefrontMesh("../data/ship/MagneViking.obj");
    auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
    shape->SetMesh(mesh);
    ship->AddAsset(shape);


    ship->SetCollide(false); // TODO: essayer avec..

    // Adding a color to the ship
//    auto color = std::make_shared<chrono::ChColorAsset>();
//    color->SetColor(chrono::ChColor(255, 255, 0));
//    ship->AddAsset(color);

    // Creating a force
    auto force = std::make_shared<frydom::FrTryalForce>();

    force->SetName("essai_force");

//    force->

    ship->AddForce(force);


    // Adding the ship to the system
    system.AddBody(ship);


    // Printing hierarchy
    system.ShowHierarchy(chrono::GetLog());

//    system.SetSolverType();

    // Visualization with irrlicht
    if (viz) {

        // Using own class for irrlicht viz
        frydom::FrIrrApp app(&system, L"Frydom vizualization based on Irrlicht");
        app.AddTypicalLights();
        app.AddTypicalCamera(irr::core::vector3df(0, 0, 300), irr::core::vector3df(1, 0, -1));


        app.AssetBindAll();
        app.AssetUpdateAll();


//        app.SetStepManage(true);
        app.SetTimestep(step_size);
        app.SetTryRealtime(true);



        while (app.GetDevice()->run()) {
            app.BeginScene();
            app.DrawAll();
            app.DoStep();
            app.EndScene();
//            std::cout << "End step " << system.GetTimestepper()->GetTime() << std::endl;
        }

    }


    std::cout << "LEAVING MAIN PROGRAM" << "\n";
    return 0;

}
