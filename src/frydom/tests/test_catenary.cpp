//
// Created by frongere on 03/08/17.
//

#include <frydom/catenary/FrCatenaryLine.h>


using namespace frydom;
//using namespace environment;

int main(int argc, char* argv[]) {


    // Creating two nodes

    auto node1 = std::make_shared<FrCatenaryNode>();
    auto node2 = std::make_shared<FrCatenaryNode>(50, 0, 0);

    auto line = FrCatenaryLine(node1, node2);

    line.UpdateCache();




    return 0;
}