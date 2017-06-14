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

#include <math.h>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "../core/FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"
#include "../utils/FrIrrApp.h"

#include "../hydrodynamics/FrITTC57.h"
#include "../hydrodynamics/FrTryalForce.h"


#include <irrlicht.h>
#include <chrono/physics/ChLinkMate.h>

bool viz = true;
float friction = 0.6f;
double step_size = 1e-2;
bool capture_video = false;



int main(int argc, char* argv[]) {

    // Creating the system
    auto system = frydom::FrOffshoreSystem();

    // Creating the free surface and assigning it to a unique pointer as we should have only one free surface that has to be owned by the OffshoreSystem
    auto free_surface = std::make_unique<frydom::environment::FrFlatFreeSurface>(0.);
    free_surface->Initialize(-800, 800, 400);

    // Giving the free surface's ownership to the system (it becomes responsible of the destruction)
    system.setFreeSurface(free_surface.release()); // le release effectue un transfert de propriete de free_surface a system qui devient responsable de la destruction


    // Contact method
//    chrono::ChMaterialSurface::ContactMethod contactMethod = chrono::ChMaterialSurface::SMC;


    // Creating a body that has to be a floating body
    auto ship = std::make_shared<chrono::ChBody>();
    ship->SetName("my_ship");
//    auto ship = std::make_shared<chrono::ChBody>(chrono::ChMaterialSurface::NSC);
    ship->SetIdentifier(1);
    double mass = 5e6;
    ship->SetMass(mass);
    ship->SetPos(chrono::ChVector<>(-200, 0, 0));

    auto rot = chrono::Q_from_AngAxis(30.*M_PI/180., chrono::ChVector<>(0, 0, 1));
//    auto rot = chrono::ChQuaternion<>(, );


    ship->SetRot(rot);

    auto ship_velocity = ship->TransformDirectionLocalToParent(chrono::ChVector<>(4, 0, 0));
    ship->SetPos_dt(ship_velocity);
//    ship->SetRot_dt(chrono::ChQuaternion<>(0.3, chrono::ChVector<>(0, 1, 0)));


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
    // Adding the ship to the system
    system.AddBody(ship);

    ///===========================================================================================================
    /// FORCES
    ///===========================================================================================================

    // Adding a color to the ship
//    auto color = std::make_shared<chrono::ChColorAsset>();
//    color->SetColor(chrono::ChColor(255, 255, 0));
//    ship->AddAsset(color);

    // Creating a force
//    auto force = std::make_shared<frydom::FrTryalForce>();
//    ship->AddForce(force); // Toujours ajouter la force au corps avant de la tuner !!!
//    force->SetName("essai_force");
//    force->SetMforce(9.81*mass);
//    force->SetDir(chrono::ChVector<>(0, 0, 1));

    // Creating a "propulsion force"
    auto force2 = std::make_shared<frydom::FrTryalForce>();
    ship->AddForce(force2); // Toujours ajouter la force au corps avant de la tuner !!!
//    force2->SetName("essai_force");
//    force2->SetMforce(1e7);
//    force2->SetDir(chrono::ChVector<>(1, 0, 0));


    // Creating an ITTC57 force
    auto force_ittc = std::make_shared<frydom::FrITTC57>();
    ship->AddForce(force_ittc);
    force_ittc->SetCharacteristicLength(80.);
    force_ittc->SetHullFormFactor(0.1);
    force_ittc->SetHullWettedSurface(2700.);


    ///===========================================================================================================
    /// 3 DOF CONSTRAINT
    ///===========================================================================================================

//    // Creating a constraint plane/plane (Making the ship 3 DOF)  LOCK FORMULATION
//    auto plane_constraint = std::make_shared<chrono::ChLinkLockPlanePlane>();
//    auto fs_body = system.getFreeSurface()->getBody();
//    plane_constraint->Initialize(ship, fs_body, chrono::ChCoordsys<>(chrono::ChVector<>(0, 0, 0)));
//    system.AddLink(plane_constraint);  // FIXME: ne fonctionne pas

    // Creating a constraint plane/plane (Making the ship 3 DOF)  LOCK FORMULATION
    // TODO: avoir une methode make 3DOF pour la classe Ship
    auto plane_constraint = std::make_shared<chrono::ChLinkMatePlane>();
    auto fs_body = system.getFreeSurface()->getBody();
    plane_constraint->Initialize(ship, fs_body,
                                 true,
                                 chrono::ChVector<>(),
                                 chrono::ChVector<>(),
                                 chrono::ChVector<>(0, 0, -1),
                                 chrono::ChVector<>(0, 0, 1));  // FIXME: pourquoi on est obliges de renverser la normale ?

    system.AddLink(plane_constraint);  // FIXME: ne fonctionne pas



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

        app.SetVideoframeSave(capture_video);


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
