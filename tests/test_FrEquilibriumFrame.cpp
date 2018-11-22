//
// Created by camille on 22/11/18.
//

#include "frydom/frydom.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

using namespace frydom;

int main(int argc, char* argv[]) {


    FrOffshoreSystem_ system;

    auto body = system.NewBody();

    auto eqFrame = std::make_shared<FrEqFrameMeanMotion_>(body.get(), 60., 0.1);
    system.AddPhysicsItem(eqFrame);

    system.Initialize();

    double time;
    double dt = 0.01;

    system.SetTimeStep(dt);

    while (time < 20.) {

        system.AdvanceTo(time);
        time += dt;

    }


}