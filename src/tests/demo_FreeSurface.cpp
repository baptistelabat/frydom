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

#include "core/FrConstants.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "core/FrOffshoreSystem.h"
#include "core/FrShip.h"
#include "environment/waves/FrFlatFreeSurface.h"

#include "environment/current/FrCurrentForce.h"

#include "utils/FrIrrApp.h"
#include "core/FrForceAsset.h"

#include "hydrodynamics/FrITTC57.h"
#include "hydrodynamics/FrTryalForce.h"


#include <irrlicht.h>
#include <chrono/physics/ChLinkMate.h>

bool viz = true;
float friction = 0.6f;
double step_size = 1e-2;
bool capture_video = false;



int main(int argc, char* argv[]) {

    // =================================================================================================================
    // OFFSHORE SYSTEM
    // =================================================================================================================
    auto system = frydom::FrOffshoreSystem();

    // =================================================================================================================
    // FREE SURFACE
    // =================================================================================================================
    // Creating the free surface and assigning it to a unique pointer as we should have only one free surface that has to be owned by the OffshoreSystem
    auto free_surface = std::make_unique<frydom::environment::FrFlatFreeSurface>(0.);
    free_surface->Initialize(-400, 400, 100);

    // Giving the free surface's ownership to the system (it becomes responsible of the destruction)
    system.setFreeSurface(free_surface.release()); // le release effectue un transfert de propriete de free_surface a system qui devient responsable de la destruction

    // =================================================================================================================
    // CURRENT
    // =================================================================================================================
    // Creating a current field
    auto current_field = std::make_unique<frydom::environment::FrCurrent>(frydom::EAST,
                                                                          5,
                                                                          frydom::KNOT);


    system.setCurrent(current_field.release());



//    auto current = system.GetCurrent();
//    auto dir_ned = current->getDirection(frydom::NED);
//    auto dir_nwu = current->getDirection(frydom::NWU);


    // Contact method
//    chrono::ChMaterialSurface::ContactMethod contactMethod = chrono::ChMaterialSurface::SMC;

    // =================================================================================================================
    // SHIP
    // =================================================================================================================
    // Creating a body that has to be a floating body
    auto ship = std::make_shared<frydom::FrShip>();

//    auto ship = std::make_shared<chrono::ChBody>();
    ship->SetName("my_ship");
//    auto ship = std::make_shared<chrono::ChBody>(chrono::ChMaterialSurface::NSC);
    ship->SetIdentifier(1);
    double mass = 5e6;
    ship->SetMass(mass);
    ship->SetPos(chrono::ChVector<>(-200, 0, 0));

    auto rot = chrono::Q_from_AngAxis(0.*M_PI/180., chrono::ChVector<>(0, 0, 1));
//    auto rot = chrono::ChQuaternion<>(, );


    ship->SetRot(rot);

    auto ship_velocity = ship->TransformDirectionLocalToParent(chrono::ChVector<>(5, 0, 0));
    ship->SetPos_dt(ship_velocity);
//    ship->SetRot_dt(chrono::ChQuaternion<>(0.3, chrono::ChVector<>(0, 1, 0)));


    ship->SetBodyFixed(false); // TODO: debloquer
//    auto material = std::make_shared<chrono::ChMaterialSurfaceNSC>();
//    ship->SetMaterialSurface(material);auto

    ship->SetInertiaXX(chrono::ChVector<>(1e5, 5e6, 5e6));

    // Defining the hydro mesh and as an asset
    ship->SetHydroMesh("../data/ship/MagneViking.obj", true);


    ship->SetCollide(false); // TODO: essayer avec..
    // Adding the ship to the system
    system.AddBody(ship);

    // =================================================================================================================
    // FORCES
    // =================================================================================================================

    // Creating a "propulsion force"
//    auto force2 = std::make_shared<frydom::FrTryalForce>();
//    ship->AddForce(force2); // Toujours ajouter la force au corps avant de la tuner !!!
//    auto force_asset = std::make_shared<frydom::FrForceAsset>(force2);
//    ship->AddAsset(force_asset);



    // Creating an ITTC57 force
//    auto force_ittc = std::make_shared<frydom::FrITTC57>();
//    ship->AddForce(force_ittc);
//    force_ittc->SetCharacteristicLength(80.);
//    force_ittc->SetHullFormFactor(0.1);
//    force_ittc->SetHullWettedSurface(2700.);


    // Creating a current force
//    auto current_force = std::make_shared<frydom::FrCurrentForce>();
//    ship->AddForce(current_force);


    ///===========================================================================================================
    /// 3 DOF CONSTRAINT
    ///===========================================================================================================

    ship->Set3DOF_ON();

    ///===========================================================================================================
    /// VISUALIZATION WITH IRRLICHT
    ///===========================================================================================================

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

            std::cout << ship->GetPos_dt().x() << std::endl;

//            std::cout << "End step " << system.GetTimestepper()->GetTime() << std::endl;
        }

    }


    std::cout << "LEAVING MAIN PROGRAM" << "\n";
    return 0;

}
