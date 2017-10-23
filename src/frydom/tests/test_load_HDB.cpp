//
// Created by frongere on 23/10/17.
//

#include "frydom/hydrodynamics/FrHydroDB.h"


using namespace frydom;

int main(int argc, char* argv[]) {


    auto IRFDB = LoadIRFData("../src/frydom/tests/data/hydro_db.yml", "Cylinder_hdb");


    return 0;
}
