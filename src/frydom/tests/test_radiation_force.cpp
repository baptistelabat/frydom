//
// Created by frongere on 20/10/17.
//

#include "frydom/hydrodynamics/FrRadiationForce.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    // Building impulse response function database
    auto IRFDB = FrRadiationIRFDB(6, 6);

    // Building radiation force based on this DB
    auto radiation_force = FrRadiationConvolutionForce(IRFDB);




    return 0;
}