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

#include "frydom/frydom.h"

bool viz = true;
float friction = 0.6f;
double step_size = 1e-2;
bool capture_video = false;

using namespace frydom;

int main(int argc, char* argv[]) {

    // =================================================================================================================
    // OFFSHORE SYSTEM
    // =================================================================================================================
    auto system = FrOffshoreSystem();

    // =================================================================================================================
    // FREE SURFACE
    // =================================================================================================================
    system.GetEnvironment()->GetFreeSurface()->SetGrid(-200, 200, 50);

    // =================================================================================================================
    // CURRENT
    // =================================================================================================================
    // Creating a current field
    auto current_field = std::make_unique<FrCurrent>(frydom::EAST, 5, KNOT);


    system.GetEnvironment()->SetCurrent(current_field.release());


    // =================================================================================================================
    // SHIPS
    // =================================================================================================================

    // ------------------------------------
    // SHIP1
    // ------------------------------------

    // Creating a body that has to be a floating body
    auto ship1 = std::make_shared<FrShip>();

    ship1->SetName("Ship1");
    ship1->SetIdentifier(1);
    double mass = 5e6;
    ship1->SetMass(mass);
    ship1->SetPos(chrono::ChVector<>(-200, 50, 0));

    auto rot = chrono::Q_from_AngAxis(radians(-2.), chrono::ChVector<>(0, 0, 1));

    ship1->SetRot(rot);

    // TODO: creer des methodes pour imposer simplement la position, vitesse, acceleration... dans le repere qu'on veut
    // (local ou global) simplement sans avoir a specifier d'objets de l'API chrono !!!
    // SetPosition(x, y, z, frame={local, global}
    // SetVelocity(vx, vy, vz, frame={local, global})
    // SetAcceleration(ax, ay, az) ...

    auto ship_velocity = ship1->TransformDirectionLocalToParent(chrono::ChVector<>(12, 0, 0));
    ship1->SetPos_dt(ship_velocity);

    ship1->SetBodyFixed(false); // TODO: debloquer --> doit etre dans le constructeur
    ship1->SetInertiaXX(chrono::ChVector<>(1e5, 5e6, 5e6));

    // Defining the hydro mesh and as an asset
    ship1->SetHydroMesh("MagneViking.obj", true);

    ship1->SetCollide(false); // TODO: essayer avec..
    // Adding the ship1 to the system
    system.AddBody(ship1);

    ship1->Set3DOF_ON();

    // ------------------------------------
    // SHIP2
    // ------------------------------------

    // Creating a body that has to be a floating body
    auto ship2 = std::make_shared<FrShip>();

    ship2->SetName("Ship2");
    ship2->SetIdentifier(2);
    ship2->SetMass(mass);
    ship2->SetPos(chrono::ChVector<>(-150, -50, 0));

    auto rot2 = chrono::Q_from_AngAxis(3.*M_PI/180., chrono::ChVector<>(0, 0, 1));
    ship2->SetRot(rot2);

    auto ship_velocity2 = ship2->TransformDirectionLocalToParent(chrono::ChVector<>(8, 0, 0));
    ship2->SetPos_dt(ship_velocity2);

    ship2->SetBodyFixed(false); // TODO: --> mettre dans le constructeur !!
    ship2->SetInertiaXX(chrono::ChVector<>(1e5, 5e6, 5e6));

    // Defining the hydro mesh and as an asset
    ship2->SetHydroMesh("MagneViking.obj", true);

    ship2->SetCollide(false); // TODO: essayer avec..
    // Adding the ship1 to the system
    system.AddBody(ship2);

    ship2->Set3DOF_ON();


    // ------------------------------------
    // SHIP3
    // ------------------------------------

    // Creating a body that has to be a floating body
    auto ship3 = std::make_shared<FrShip>();

    ship3->SetName("Ship3");
    ship3->SetIdentifier(5);
    ship3->SetMass(mass);
    ship3->SetPos(chrono::ChVector<>(0, 0, 0));

    auto rot3 = chrono::Q_from_AngAxis(0.*M_PI/180., chrono::ChVector<>(0, 0, 1));
    ship3->SetRot(rot3);

    auto ship_velocity3 = ship3->TransformDirectionLocalToParent(chrono::ChVector<>(4, 0, 0));
    ship3->SetPos_dt(ship_velocity3);

    ship3->SetBodyFixed(false); // TODO: debloquer
    ship3->SetInertiaXX(chrono::ChVector<>(1e5, 5e6, 5e6));

    // Defining the hydro mesh and as an asset
    ship3->SetHydroMesh("MagneViking.obj", true);

    ship3->SetCollide(false); // TODO: essayer avec..
    // Adding the ship1 to the system
    system.AddBody(ship3);

    ship3->Set3DOF_ON();




    // =================================================================================================================
    // FORCES
    // =================================================================================================================

    // Creating a "propulsion force"
//    auto force2 = std::make_shared<frydom::FrTryalForce>();
//    ship1->AddForce(force2); // Toujours ajouter la force au corps avant de la tuner !!!
//    auto force_asset = std::make_shared<frydom::FrForceAsset>(force2);
//    ship1->AddAsset(force_asset);



    // Creating an ITTC57 force
//    auto force_ittc = std::make_shared<frydom::FrITTC57>();
//    ship1->AddForce(force_ittc);
//    force_ittc->SetCharacteristicLength(80.);
//    force_ittc->SetHullFormFactor(0.1);
//    force_ittc->SetHullWettedSurface(2700.);


    // Creating a current force
//    auto current_force = std::make_shared<frydom::FrCurrentForce>();
//    ship1->AddForce(current_force);


    ///===========================================================================================================
    /// 3 DOF CONSTRAINT
    ///===========================================================================================================



    ///===========================================================================================================
    /// VISUALIZATION WITH IRRLICHT
    ///===========================================================================================================

    // Visualization with irrlicht

    // TODO: utiliser l'objet frydom pourla visu irrlicht
    if (viz) {

        // Using own class for irrlicht viz
        FrIrrApp app(&system, L"Frydom vizualization based on Irrlicht");
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
        }

    }


    return 0;

}
