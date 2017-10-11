//
// Created by frongere on 11/10/17.
//

#include "frydom/cable/FrDynamicCable.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    auto cable = FrDynamicCable();

    cable.SetCableLength(50);
    cable.SetLinearDensity(200);
    cable.SetDiameter(0.1);
    cable.SetNumberOfElements(50);
    cable.SetYoungModulus(1e9);

    auto body1 = std::make_shared<FrBody>();
    body1->SetBodyFixed(true);

    auto node1 = body1->CreateNode(chrono::ChVector<double>(0, 0, 0));

    auto node2 = body1->CreateNode(chrono::ChVector<double>(20, 0, 0));

    cable.SetStartingNode(node1);
    cable.SetEndingNode(node2);



    cable.Initialize();



    return 0;
}