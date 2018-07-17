//
// Created by camille on 17/07/18.
//

#include "frydom/frydom.h"

using namespace frydom;
using namespace chrono;

int main(int argc, char* argv[]) {

    // System
    FrOffshoreSystem system;

    // Environment
    system.GetEnvironment()->SetCurrent(FrCurrent::UNIFORM);
    system.GetEnvironment()->GetCurrent()->Set(NORTH, 10., KNOT, NED, GOTO);

    // Body
    auto mybody = std::make_shared<FrHydroBody>();
    mybody->SetRot(euler_to_quat(chrono::ChVector<>(0., 0., 0.)));
    system.Add(mybody);

    // Drag force
    auto drag_force = std::make_shared<FrCurrentStandardForce>();
    drag_force->SetWaterDensity(1025.);
    drag_force->SetMaxBreadth(2.);
    drag_force->SetDraft(1.);
    drag_force->SetLpp(10.);
    drag_force->SetXc(0.);
    drag_force->SetLateralArea(10.);

    mybody->AddForce(drag_force);

    // Initialize
    system.Initialize();

    // Output
    drag_force->UpdateState();
    std::cout << "F.x : " << drag_force->GetForce().x() << std::endl;

}