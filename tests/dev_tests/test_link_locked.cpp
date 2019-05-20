//
// Created by camille on 13/05/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    std::cout << " ========================================= Test Link Locked ========================== " << std::endl;

    // System

    FrOffshoreSystem system;

    // Bodies

    auto body1 = make_BoxBody(10, 10, 4, 100.);
    body1->GetDOFMask()->MakeItLocked();
    system.AddBody(body1);

    auto node1 = body1->NewNode();
    node1->SetPositionInBody(Position(0., 0., -2), NWU);

    auto body2 = make_BoxBody(5, 5., 2, 12.5);
    body2->SetPosition(Position(0., 0., -3), NWU);
    system.AddBody(body2);

    body2->SetAngularVelocityInWorld(AngularVelocity(0., 0., 0.2), NWU);

    auto node2 = body2->NewNode();
    node2->SetPositionInBody(Position(0., 0., 1.), NWU);

    // Link

    auto link = make_revolute_link(node1, node2, &system);
    //auto motor = link->Motorize(ACTUATOR_CONTROL::VELOCITY);
    //motor->SetMotorFunction(FrConstantFunction(0.2));

    // Simulation

    double dt = 0.01;

    system.SetTimeStep(dt);

    system.Initialize();

    double time = 0.;
    double phi, theta, psi;



    while (time < 40.) {

        time += dt;
        system.AdvanceTo(time);


        if (std::abs(time - 15.) < 0.1*dt) {
            link->SetLocked(true);
        }

        body2->GetRotation().GetCardanAngles_DEGREES(phi, theta, psi, NWU);
        std::cout << " time : " << time << " s ; rotation : " << psi << " deg." << std::endl;

    }



    std::cout << " ================================================ End ================================ " << std::endl;
}