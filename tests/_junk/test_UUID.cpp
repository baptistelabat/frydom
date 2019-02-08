//
// Created by frongere on 11/01/18.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    auto myObj1 = FrObject();
    std::cout << myObj1.GetUUID() << "\n";

    auto myObj2 = FrObject();
    std::cout << myObj2.GetUUID() << "\n";




}