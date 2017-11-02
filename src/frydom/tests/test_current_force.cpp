//
// Created by frongere on 10/07/17.
//

//#include "frydom/environment/FrEnvironment.h"
#include "frydom/IO/FrLoader.h"
#include "frydom/environment/current/FrCurrentPolarCoeffs.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    // Create an offshore system


    // Set the current field


    // Building a coefficients table
    auto table = IO::MakeCurrentPolarCoeffTable("../src/frydom/tests/data/PolarCurrentCoeffs.yml");





    return 0;
};