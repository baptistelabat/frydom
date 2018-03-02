//
// Created by frongere on 04/07/17.
//

#include "../misc/FrLinspace.h"

int main(int argc, char* argv[]) {

    auto x1 = frydom::arange<double>(10, 2, 0);
    auto x2 = frydom::linspace<double>(10, 0, 22, true);

    for (int i=0; i<x2.size(); i++) {
        std::cout << x2.at(i) << std::endl;
    }

    std::cout << "\t" << x2.size() << std::endl;

}