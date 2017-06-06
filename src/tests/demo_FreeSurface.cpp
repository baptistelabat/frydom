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

#include <irrlicht.h>

bool viz = true;
float friction = 0.6f;

int main(int argc, char* argv[]) {


    // Creating the system
    auto system = frydom::FrOffshoreSystem();

    // Creating the free surface and assigning it to a unique pointer as we should have only one free surface
    auto free_surface = std::make_unique<frydom::environment::FrFlatFreeSurface>(0.);
    free_surface->Initialize(-100, 100, 25, -20, 20, 20);

    // Giving the free surface's ownership to the system (it becomes responsible of the destruction)
    system.setFreeSurface(free_surface.release()); // le release effectue un transfert de propriete de free_surface a system qui devient responsable de la destruction


    // Creating a body that has to be a floating body
    auto ship = std::make_shared<chrono::ChBody>(chrono::ChMaterialSurface::NSC);
    ship->SetIdentifier(1);
    ship->SetMass(5e6);
    ship->SetPos(chrono::ChVector<>(0, 0, 9));
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


    // Setting the collision model
    ship->GetMaterialSurfaceNSC()->SetFriction(friction);




    ship->GetCollisionModel()->ClearModel();
    ship->GetCollisionModel()->AddTriangleMesh(mesh, false, false);
    ship->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.001);
    ship->GetCollisionModel()->SetDefaultSuggestedMargin(0.0005);
    ship->GetCollisionModel()->BuildModel();
    ship->SetCollide(true); // TODO: essayer avec..

    // Adding a color to the ship
//    auto color = std::make_shared<chrono::ChColorAsset>();
//    color->SetColor(chrono::ChColor(255, 255, 0));
//    ship->AddAsset(color);


    // Adding the ship to the system
    system.AddBody(ship);




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
        app.SetTimestep(0.01);
        app.SetTryRealtime(true);



        while (app.GetDevice()->run()) {
            app.BeginScene();
            app.DrawAll();
            app.DoStep();
            app.EndScene();
            std::cout << "End step " << system.GetTimestepper()->GetTime() << std::endl;
        }

    }


    std::cout << "LEAVING MAIN PROGRAM" << "\n";
    return 0;

}
