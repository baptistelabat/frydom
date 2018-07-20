//
// Created by Lucas Letournel on 18/07/18.
//


#include <chrono/physics/ChSystemNSC.h>
#include "frydom/frydom.h"
#include <fmt/format.h>

using namespace chrono;
using namespace frydom;

using namespace chrono::irrlicht;
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

int main(){
    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem my_system;

    // --------------------------------------------------
    // Bodies
    // --------------------------------------------------

    std::shared_ptr<FrBody> base;
    base = std::make_shared<FrSphere>(0.1,1000, true);
    base->SetName("base");
    base->SetPos(ChVector<>(0,0,10));
    base->SetBodyFixed(true);
    my_system.Add(base);

    std::shared_ptr<ChBody> pendulum1;
    pendulum1 = std::make_shared<FrSphere>(1,1000, true);
    pendulum1->SetName("pendulum");
    pendulum1->SetPos(ChVector<>(1.2,0,10));
    my_system.Add(pendulum1);

    std::shared_ptr<ChBody> pendulum2;
    pendulum2 = std::make_shared<FrSphere>(1,1000, true);
    pendulum2->SetName("pendulum");
    pendulum2->SetPos(ChVector<>(3.3,0,10));
    my_system.Add(pendulum2);

    std::shared_ptr<ChBody> pendulum3;
    pendulum3 = std::make_shared<FrSphere>(1,1000, true);
    pendulum3->SetName("pendulum");
    pendulum3->SetPos(ChVector<>(5.5,0,10));
    my_system.Add(pendulum3);

    // --------------------------------------------------
    // Links
    // --------------------------------------------------

    auto link1 = std::make_shared<ChLinkLockRevolute>();
    link1->Initialize(pendulum1, base, ChCoordsys<>(ChVector<>(0, 0, 10), CH_C_PI_2, VECT_X));
    my_system.AddLink(link1);

    auto link2 = std::make_shared<ChLinkLockRevolute>();
    link2->Initialize(pendulum2, pendulum1, ChCoordsys<>(ChVector<>(2.2, 0, 10), CH_C_PI_2, VECT_X));
    my_system.AddLink(link2);

    auto link3 = std::make_shared<ChLinkLockRevolute>();
    link3->Initialize(pendulum3, pendulum2, ChCoordsys<>(ChVector<>(4.4, 0, 10), CH_C_PI_2, VECT_X));
    my_system.AddLink(link3);

    // -----------------------------------------------
    // Simulation
    // -----------------------------------------------

    double dt = 0.01;

    my_system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    my_system.SetStep(dt);
    my_system.Initialize();

    auto app = FrIrrApp(my_system);
    app.AddTypicalCamera(irr::core::vector3df(1, 1, 25), irr::core::vector3df(0, 0, 10));
    app.Run();


}