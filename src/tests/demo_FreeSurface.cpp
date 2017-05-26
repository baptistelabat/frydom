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

#include <iostream>
#include <unistd.h>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "../environment/waves/FrFlatFreeSurface.h"

#include <irrlicht.h>


// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main(int argc, char* argv[]) {
    // Creating the system
    ChSystemNSC system;

    // Creating the free surface
    frydom::FrFlatFreeSurface free_surface(2);

    free_surface.Initialize(0, 1000, 1);


    chrono::geometry::ChTriangleMeshConnected mesh = free_surface.getMesh();

    chrono::ChVector<>* vertex;

//    int nv = mesh.getCoordsVertices().size();

    // TODO : voir iterator


//    for (int k = 0; k < mesh.getCoordsVertices().size(); k++){
//        vertex = &mesh.m_vertices[k];
////        std::cout << vertex[0] << "\t" << vertex[1] << "\t" << vertex[2] << std::endl;
//    }




//    // Creating the irrlicht 3D application
//    ChIrrApp application(&system, L"Demo_FlatFreeSurface",
//                         core::dimension2d<u32>(800, 600),
//                         false, false, true);
//
//
//    application.AddTypicalLogo();
//    application.AddTypicalSky();
//    application.AddTypicalLights();
//    application.AddTypicalCamera();
//
//
//    // Creating a body
//    auto body = std::make_shared<ChBody>();
//    body->SetBodyFixed(true);
//
//    // Define a collision shape
//    body->GetCollisionModel()->ClearModel();
//    body->GetCollisionModel()->AddBox(10, 0.5, 10, ChVector<>(0, -1, 0));
//    body->GetCollisionModel()->BuildModel();
//    body->SetCollide(true);
//
//
//    // Add body to system
////    application.GetSystem()->Add(body);
//    system.Add(body);
//
//    // Building an asset to be represented
//    auto body_box = std::make_shared<ChBoxShape>();
//    body_box->GetBoxGeometry().Pos = ChVector<>(0, -1, 0);
//    body_box->GetBoxGeometry().Size = ChVector<>(10, 0.5, 10);
//    body->AddAsset(body_box);
//
//
//    // Bind assets
//    application.AssetBindAll();
//    application.AssetUpdateAll();
//
//    //
//    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
//    //
//
//    // This will help choosing an integration step which matches the
//    // real-time step of the simulation..
//    ChRealtimeStepTimer m_realtime_timer;
//
////    bool removed = false;
//
//    while (application.GetDevice()->run()) {
//        // Irrlicht must prepare frame to draw
//        application.BeginScene(true, true, SColor(255, 140, 161, 192));
//
//        // Irrlicht now draws simple lines in 3D world representing a
//        // skeleton of the mechanism, in this instant:
//        //
//        // .. draw items belonging to Irrlicht scene, if any
//        application.DrawAll();
//        // .. draw a grid
//        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.5, 0.5);
//        // .. draw GUI items belonging to Irrlicht screen, if any
//        application.GetIGUIEnvironment()->drawAll();
//
//        // HERE CHRONO INTEGRATION IS PERFORMED: THE
//        // TIME OF THE SIMULATION ADVANCES FOR A SINGLE
//        // STEP:
//
//        system.DoStepDynamics(m_realtime_timer.SuggestSimulationStep(0.02));
//
//        // Irrlicht must finish drawing the frame
//        application.EndScene();
//
//        usleep(10);
//    }

    return 0;

}

