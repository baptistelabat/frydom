//
// Created by frongere on 07/09/2017.
//

#include "frydom/core/FrCore.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/catenary/FrCatenaryLine.h"

#include "irrlicht.h"
#include "frydom/utils/FrIrrApp.h"

using namespace frydom;
using namespace environment;

bool viz = true;
//float friction = 0.6f;
double step_size = 1e-2;
bool capture_video = false;



int main(int argc, char* argv[]) {

    // The system
    FrOffshoreSystem system;

    // Set the free surface
    auto free_surface = std::make_unique<frydom::environment::FrFlatFreeSurface>(0.);
    free_surface->Initialize(-400, 400, 200, -100, 100, 100);
    system.setFreeSurface(free_surface.release());


    // The current
    auto current_field = std::make_unique<FrCurrent>(WEST, 1, KNOT, NED, COMEFROM);
    // TODO: changer pour faire des move plutot que des release...
    system.setCurrent(current_field.release());

    // Building a TUG
    auto tug = std::make_shared<FrShip>();
    system.AddBody(tug);
    tug->SetPos(chrono::ChVector<>(0, 0, 0));
    tug->Set3DOF_ON();
    tug->SetHydroMesh("../data/ship/MagneViking.obj", true);
    tug->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    tug->SetMass(5);

    // Adding a resistance
//    std::string filename("../src/frydom/tests/data/PolarCurrentCoeffs.yml");
//    auto current_force = std::make_shared<FrCurrentForce>(filename);
//    tug->AddForce(current_force);



    // Creating a fairlead on the tug
    auto fairlead = tug->CreateNode(chrono::ChVector<>(-41, 0, 8.18));
    fairlead->SetName("Fairlead");

    // Creating an other node as an anchor
    auto anchor = std::make_shared<FrNode>();
    anchor->SetBody(system.GetWorldBodyPtr());  // Faire une classe anchor qui sait le faire toute seule

    auto anchor_pos = chrono::ChCoordsys<double>();
    anchor_pos.pos.x() = 100;

    anchor->Impose_Abs_Coord(anchor_pos);



    // Creating a catenary line
    double Lu = 220;
    bool elastic = true;
    auto u = chrono::ChVector<double>(0, 0, -1);
    double q = 616.538;
    double EA = 1.5708e9;
    auto line = FrCatenaryLine(fairlead, anchor, elastic, EA, Lu, q, u);


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

//        app.SetVideoframeSave(capture_video);


        auto fairlead_coords = fairlead->GetAbsPos();
        std::cout << fairlead_coords[0] << "\t" << fairlead_coords[1] << std::endl;

        while (app.GetDevice()->run()) {
            app.BeginScene();
            app.DrawAll();
            app.DoStep();
            app.EndScene();

            fairlead_coords = fairlead->GetAbsPos();
            std::cout << fairlead_coords[0] << "\t" << fairlead_coords[1] << std::endl;

//            std::cout << ship1->GetPos_dt().x() << std::endl;

//            std::cout << "End step " << system.GetTimestepper()->GetTime() << std::endl;
        }
    }




    return 0;
}