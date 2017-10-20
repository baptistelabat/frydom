//
// Created by frongere on 20/10/17.
//

#include <memory>
#include "frydom/misc/FrEigen.h"
#include "frydom/hydrodynamics/FrRadiationForce.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    // Building impulse response function database
    auto IRFDB = FrRadiationIRFDB(6, 6);

    // Emulating impulse response functions as a sinc function
    double tf = 20.;
    unsigned int nt = 200;
    auto time = linspace<double>(0, tf, nt);

    auto Ktij = std::vector<double>();
    Ktij.reserve(nt);
    double t;
    for (unsigned int i=0; i < nt; ++i) {
        t = time[i];
        if (t == 0.) {
            Ktij.push_back(1.);
        } else {
            Ktij.push_back(sin(t) / t);
        }
        std::cout << Ktij[i] << std::endl;
    }


    // Populating the database --> A faire a la lecture d'un fichier de donnees
    IRFDB.SetTime(tf, nt);

    for (int i=0; i<6; ++i) {
        for (int j=0; j<6; ++j) {
            IRFDB.SetKernel(i, j, Ktij);
        }
    }


//    IRFDB.



    // Building radiation force based on this DB
//    auto radiation_force = FrRadiationConvolutionForce(IRFDB);




    return 0;
}