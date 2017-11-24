//
// Created by frongere on 03/08/17.
//

#include "frydom/frydom.h"

using namespace frydom;
//using namespace environment;

int main(int argc, char* argv[]) {

    // Creating a support body for nodes
    auto myBody = FrBody();

    // Creating two nodes
    auto node1 = myBody.CreateNode(chrono::ChVector<>());
    auto node2 = myBody.CreateNode(chrono::ChVector<>(100, 0, 0));

    // Line properties
    double Lu = 220;
    auto u = chrono::ChVector<double>(0, sqrt(3)*0.5, 0.5);
    double q = 616.538;
    double EA = 1.5708e9;
    double A = 0.05;
    double E = EA/A;

    auto line = FrCatenaryLine(node1, node2, true, E, A, Lu, q, u);

    line.solve();

    myBody.UpdateForces(false); // A quoi ca sert ??? (en plus ca semble pas etre un bool en param...)


    auto t0 = line.get_tension(0.);
    std::cout << t0[0] << std::endl;
    std::cout << t0[1] << std::endl;
    std::cout << t0[2] << std::endl;



//    auto xx = linspace(150, 230, 1000);
//
//    double x = 150;
//    double dx = 1.;
//    while (x < 300) {
//        std::cout << std::endl <<  "Solution pour x = " << x << std::endl;
//
//        node2->SetPos(x, 0., 0.);
//        line.solve();
//
//        std::cout << line.GetCableLength() << std::endl;
//
//        x += dx;
//    }

    return 0;
}