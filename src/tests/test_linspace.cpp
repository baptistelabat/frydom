//
// Created by frongere on 04/07/17.
//

#include "../misc/FrLinspace.h"

int main(int argc, char* argv[]) {

    auto x1 = frydom::arange<double>(0, 10, 2);
    auto x2 = frydom::linspace<double>(0, 10, 22);

    for (int i=0; i<x2.size(); i++) {
        std::cout << x2.at(i) << std::endl;
    }

    std::cout << "\t" << x2.size() << std::endl;

}