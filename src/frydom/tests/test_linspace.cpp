//
// Created by frongere on 04/07/17.
//

#include "frydom/misc/FrLinspace.h"

int main(int argc, char* argv[]) {

    auto x1 = frydom::arange<double>(0, 10, 2);
    auto x2 = frydom::linspace<double>(0, 10, 22);

    auto x3 = frydom::logspace<double>(0, 10, 1000);

    for (int i=0; i<x3.size(); i++) {
        std::cout << x3.at(i) << std::endl;
    }
}