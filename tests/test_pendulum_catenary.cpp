//
// Created by Lucas Letournel on 18/07/18.
//


#include <chrono/physics/ChSystemNSC.h>
#include "frydom/frydom.h"
#include <fmt/format.h>

using namespace chrono;
using namespace frydom;

int main() {
    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem my_system;

    // --------------------------------------------------
    // Bodies
    // --------------------------------------------------

    std::shared_ptr<FrBody> base;
    base = std::make_shared<FrSphere>(1, 1000, true);
    base->SetName("base");
    base->SetPos(ChVector<>(0, 0, 20));
    base->SetBodyFixed(true);
    my_system.Add(base);

    std::shared_ptr<FrBody> pendulum1;
    pendulum1 = std::make_shared<FrSphere>(1, 33500, true);
    pendulum1->SetName("pendulum");
    pendulum1->SetPos(ChVector<>(10., 0, 20));
    //pendulum1->SetBodyFixed(true);
    my_system.Add(pendulum1);

    // --------------------------------------------------
    // FrNodes
    // --------------------------------------------------
    // TODO: Ajouter une méthode CreateNode avec un ChFrame en entrée
    auto baseNode = base->CreateNode(ChVector<>(0,0,0));
    //baseNode->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    auto pendulumNode = pendulum1->CreateNode(ChVector<>(0,0,0));
    //pendulumNode->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    // --------------------------------------------------
    // Joints
    // --------------------------------------------------
    //make_rotoid(baseNode, pendulumNode);
    // --------------------------------------------------
    // Catenary Line
    // --------------------------------------------------
    // Line properties
    double Lu = 10;
    auto u = chrono::ChVector<double>(0, 0, -1);
    double q = 616.538;
    double EA = 1.5708e9;
    double A = 0.05;
    double E = EA/A;

    auto Catenary = std::make_shared<FrCatenaryLine>(baseNode, pendulumNode, false, E, A, Lu, q, u);
    my_system.AddLink(Catenary);

/*
    // -----------------------------------------------
    // Simulation
    // -----------------------------------------------
    my_system.Update(true);
    double chronoTime = 0;
    while (chronoTime < 5) {
        chronoTime += 0.01;
        // PERFORM SIMULATION UP TO chronoTime
        my_system.DoFrameDynamics(chronoTime);
        // Print something on the console..
        GetLog() << "Time: " << chronoTime
                 << "  x_Pendulum: " << pendulum1->GetPos().x() << "\n";
    }
*/

    double dt = 0.01;

    my_system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    my_system.SetStep(dt);
    my_system.Initialize();

    auto app = FrIrrApp(my_system);
    app.AddTypicalCamera(irr::core::vector3df(1, 1, 50), irr::core::vector3df(0, 0, 20));
    app.Run();


}