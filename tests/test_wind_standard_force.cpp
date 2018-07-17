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
    system.GetEnvironment()->SetWind(FrWind::UNIFORM);
    system.GetEnvironment()->GetWind()->Set(NORTH, 10., KNOT, NED, GOTO);

    // Body
    auto mybody = std::make_shared<FrHydroBody>();
    system.Add(mybody);

    // Drag force
    auto drag_force = std::make_shared<FrWindStandardForce>();
    drag_force->SetLateralArea(10.);
    drag_force->SetTransverseArea(1.);
    drag_force->SetLpp(10.);
    drag_force->SetXc(0.);

    mybody->AddForce(drag_force);

    // Initialize
    system.Initialize();

    // Output
    drag_force->UpdateState();
    std::cout << "F.x : " << drag_force->GetForce().x() << std::endl;

}